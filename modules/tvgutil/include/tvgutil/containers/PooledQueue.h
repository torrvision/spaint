/**
 * tvgutil: PooledQueue.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_TVGUTIL_POOLEDQUEUE
#define H_TVGUTIL_POOLEDQUEUE

#include <deque>
#include <list>
#include <stdexcept>

#include <boost/functional/value_factory.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>

#include "../numbers/RandomNumberGenerator.h"

namespace tvgutil {

namespace pooled_queue {

/**
 * \brief The values of this enumeration can be used to specify what should happen when a push is attempted on a pooled queue with an empty pool.
 */
enum PoolEmptyStrategy
{
  /** Discard the new element. */
  PES_DISCARD,

  /** Add an extra element to the pool to accommodate the new element. */
  PES_GROW,

  /** Move a random element from the queue back to the pool to accommodate the new element. */
  PES_REPLACE_RANDOM,

  /** Wait for another thread to pop an element from the queue, thereby making space for the new element. */
  PES_WAIT
};

}

/**
 * \brief An instance of an instantiation of this class template represents a queue that is backed by a pool of reusable elements.
 */
template <typename T, template <typename,typename> class PoolContainer = std::deque>
class PooledQueue
{
  //#################### NESTED TYPES ####################
public:
  /**
   * \brief An instance of this class can be used to handle the process of pushing an element onto the queue.
   */
  class PushHandler
  {
    //~~~~~~~~~~~~~~~~~~~~ PRIVATE VARIABLES ~~~~~~~~~~~~~~~~~~~~
  private:
    /** A pointer to the pooled queue on which push was called. */
    PooledQueue<T,PoolContainer> *m_base;

    /** The element that is to be pushed onto the queue (if any). */
    boost::optional<T> m_elt;

    //~~~~~~~~~~~~~~~~~~~~ CONSTRUCTORS ~~~~~~~~~~~~~~~~~~~~
  public:
    /**
     * \brief Constructs a push handler.
     *
     * \param base  A pointer to the pooled queue on which push was called.
     * \param elt   The element that is to be pushed onto the queue (if any).
     */
    PushHandler(PooledQueue<T,PoolContainer> *base, const boost::optional<T>& elt)
    : m_base(base), m_elt(elt)
    {}

    //~~~~~~~~~~~~~~~~~~~~ DESTRUCTOR ~~~~~~~~~~~~~~~~~~~~
  public:
    /**
     * \brief Completes the push by pushing the element (if any) onto the queue.
     */
    ~PushHandler()
    {
      if(m_elt) m_base->end_push(*m_elt);
    }

    //~~~~~~~~~~~~~~~~~~~~ COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ~~~~~~~~~~~~~~~~~~~~
  private:
    // Deliberately private and unimplemented.
    PushHandler(const PushHandler&);
    PushHandler& operator=(const PushHandler&);

    //~~~~~~~~~~~~~~~~~~~~ PUBLIC MEMBER FUNCTIONS ~~~~~~~~~~~~~~~~~~~~
  public:
    /**
     * \brief Gets a reference to the element that is to be pushed onto the queue (if any).
     *
     * \return  A reference to the element that is to be pushed onto the queue (if any).
     */
    boost::optional<T&> get()
    {
      return m_elt ? boost::optional<T&>(*m_elt) : boost::none;
    }
  };

  //#################### TYPEDEFS ####################
public:
  typedef boost::shared_ptr<PushHandler> PushHandler_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** A function that can be used to construct new elements (by default, the default constructor for the element type). */
  boost::function<T()> m_maker;

  /** The synchronisation mutex. */
  mutable boost::mutex m_mutex;

  /** The pool of reusable elements that backs the queue. */
  PoolContainer<T,std::allocator<T> > m_pool;

  /** A strategy specifying what should happen when a push is attempted while the pool is empty. */
  pooled_queue::PoolEmptyStrategy m_poolEmptyStrategy;

  /** A condition variable used to wait for the pool to become non-empty. */
  boost::condition_variable m_poolNonEmpty;

  /** The queue itself. */
  std::list<T> m_queue;

  /** A condition variable used to wait for the queue to become non-empty. */
  mutable boost::condition_variable m_queueNonEmpty;

  /** The random number generator to use if using the random replacement pool empty strategy. */
  boost::shared_ptr<RandomNumberGenerator> m_rng;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a pooled queue.
   *
   * \param poolEmptyStrategy A strategy specifying what should happen when a push is attempted while the pool is empty.
   */
  explicit PooledQueue(pooled_queue::PoolEmptyStrategy poolEmptyStrategy = pooled_queue::PES_GROW)
  : m_poolEmptyStrategy(poolEmptyStrategy)
  {
    if(poolEmptyStrategy == pooled_queue::PES_REPLACE_RANDOM)
    {
      m_rng.reset(new RandomNumberGenerator(12345));
    }
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Starts a push operation.
   *
   * In a pooled queue, a push operation is not instantaneous. First, the caller calls begin_push(). This returns a push handler,
   * which allows the caller to access the element (if any) that is to be pushed onto the queue. The caller then writes into this
   * element, rather than constructing a new element from scratch. Finally, the destructor of the push handler calls end_push()
   * to actually push the element onto the queue. The end_push function itself is private to simplify usage.
   *
   * \return  A push handler that will handle the process of pushing an element onto the queue.
   */
  PushHandler_Ptr begin_push()
  {
    using namespace pooled_queue;

    boost::unique_lock<boost::mutex> lock(m_mutex);

    // The first task is to make sure that the pool contains an element into which the caller can write.
    // If the pool is currently empty, we have various options: (i) prevent the push by returning a null
    // element into which to write; (ii) create a new element and add it to the pool; (iii) move a random
    // element from the queue back to the pool (thereby allowing the new element to replace it); or (iv)
    // block until another thread pops an element from the queue and re-adds it to the pool. We choose
    // between these options by specifying a pool empty strategy when the pooled queue is constructed.
    if(m_pool.empty())
    {
      switch(m_poolEmptyStrategy)
      {
        case PES_DISCARD:
        {
          return PushHandler_Ptr(new PushHandler(this, boost::none));
        }
        case PES_GROW:
        {
          m_pool.push_back(m_maker());
          break;
        }
        case PES_REPLACE_RANDOM:
        {
          const int offset = m_rng->generate_int_from_uniform(0, static_cast<int>(m_queue.size()) - 1);
          typename std::list<T>::iterator it = m_queue.begin();
          std::advance(it, offset);
          m_pool.push_back(*it);
          m_queue.erase(it);
          break;
        }
        case PES_WAIT:
        {
          while(m_pool.empty()) m_poolNonEmpty.wait(lock);
          break;
        }
      }
    }

    // At this point, the pool definitely contains at least one element, so we can simply
    // remove the first element in the pool and return it to the caller for writing.
    T elt = m_pool.front();
    m_pool.pop_front();
    return PushHandler_Ptr(new PushHandler(this, elt));
  }

  /**
   * \brief Gets whether or not the queue is empty.
   *
   * \return  true, if the queue is empty, or false otherwise.
   */
  bool empty() const
  {
    boost::lock_guard<boost::mutex> lock(m_mutex);
    return m_queue.empty();
  }

  /**
   * \brief Initialises the pool backing the queue.
   *
   * \param capacity  The initial capacity of the pool (if we're using the 'grow' strategy, this may later change).
   * \param maker     A function that can be used to construct new elements (by default, the default constructor for the element type).
   */
  void initialise(size_t capacity, const boost::function<T()>& maker = boost::value_factory<T>())
  {
    boost::lock_guard<boost::mutex> lock(m_mutex);
    m_maker = maker;
    for(size_t i = 0, size = capacity; i < size; ++i)
    {
      m_pool.push_back(maker());
    }
  }

  /**
   * \brief Gets a reference to the first element in the queue.
   *
   * Note: This will block until the queue is non-empty.
   *
   * \return  A reference to the first element in the queue.
   */
  T& peek()
  {
    boost::unique_lock<boost::mutex> lock(m_mutex);
    while(m_queue.empty()) m_queueNonEmpty.wait(lock);
    return m_queue.front();
  }

  /**
   * \brief Gets a reference to the first element in the queue.
   *
   * Note: This will block until the queue is non-empty.
   *
   * \return  A reference to the first element in the queue.
   */
  const T& peek() const
  {
    boost::unique_lock<boost::mutex> lock(m_mutex);
    while(m_queue.empty()) m_queueNonEmpty.wait(lock);
    return m_queue.front();
  }

  /**
   * \brief Pops the first element from the queue and returns it to the pool.
   *
   * Note: This will block until the queue is non-empty.
   */
  void pop()
  {
    boost::unique_lock<boost::mutex> lock(m_mutex);
    while(m_queue.empty()) m_queueNonEmpty.wait(lock);
    m_pool.push_back(m_queue.front());
    m_queue.pop_front();
    m_poolNonEmpty.notify_one();
  }

  /**
   * \brief Gets the size of the queue.
   *
   * \return  The size of the queue.
   */
  size_t size() const
  {
    boost::lock_guard<boost::mutex> lock(m_mutex);
    return m_queue.size();
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Completes a push operation by pushing the specified element onto the queue.
   *
   * Note: This is called automatically when the push handler associated with the push is destroyed.
   *
   * \param elt The element to be pushed onto the queue.
   */
  void end_push(const T& elt)
  {
    boost::lock_guard<boost::mutex> lock(m_mutex);
    m_queue.push_back(elt);
    m_queueNonEmpty.notify_one();
  }
};

}

#endif
