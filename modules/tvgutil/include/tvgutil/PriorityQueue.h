/**
 * tvgutil: PriorityQueue.h
 */

#ifndef H_TVGUTIL_PRIORITYQUEUE
#define H_TVGUTIL_PRIORITYQUEUE

#include <map>
#include <stdexcept>
#include <vector>

#include <boost/serialization/serialization.hpp>

namespace tvgutil {

/**
 * \brief This is an implementation of priority queues that allows the keys of queue elements to be updated in-place.
 *
 * To do this, it maintains a dictionary (that allows elements to be looked up) alongside the usual heap-based
 * priority queue implementation.
 *
 * This is a more general implementation than that provided in the C++ Standard Library (namely std::priority_queue,
 * in the <queue> header, which doesn't support in-place key-updating). Algorithms that have no use for key-updating
 * may want to use the standard implementation instead of this one, since it is wasteful to maintain the dictionary
 * for no reason.
 *
 * \tparam ID   The element ID type (used for the lookup)
 * \tparam Key  The key type (the type of the priority values used to determine the element order)
 * \tparam Data The auxiliary data type (any information clients might wish to store with each element)
 * \tparam Comp A predicate specifying how the keys should be compared (the default predicate is std::less<Key>,
 *              which specifies that elements with smaller keys will be extracted first)
 */
template <typename ID, typename Key, typename Data, typename Comp = std::less<Key> >
class PriorityQueue
{
  //#################### NESTED CLASSES ####################
public:
  /**
   * \brief Each element of the priority queue stores its ID, its key and potentially some auxiliary data that may be useful to client code.
   *
   * Its auxiliary data may be changed by the client, but its key may only be changed via the priority queue's update_key() method.
   */
  class Element
  {
  private:
    ID m_id;
    Key m_key;
    Data m_data;

  public:
    Element() {}
    Element(const ID& id, const Key& key, const Data& data) : m_id(id), m_key(key), m_data(data) {}

    Data& data()            { return m_data; }
    const ID& id() const    { return m_id; }
    const Key& key() const  { return m_key; }

    friend class PriorityQueue;

    //~~~~~~~~~~~~~~~~~~~~ SERIALIZATION ~~~~~~~~~~~~~~~~~~~~

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
      ar & m_id;
      ar & m_key;
      ar & m_data;
    }

    friend class boost::serialization::access;
  };

  //#################### TYPEDEFS ####################
private:
  typedef std::map<ID,size_t> Dictionary; // maps IDs to their current position in the heap
  typedef std::vector<Element> Heap;

  //#################### PRIVATE VARIABLES ####################
private:
  // Datatype Invariant: m_dictionary.size() == m_heap.size()
  Dictionary m_dictionary;
  Heap m_heap;

  //#################### PUBLIC METHODS ####################
public:
  /**
   * \brief Clears the priority queue.
   */
  void clear()
  {
    m_dictionary.clear();
    Heap().swap(m_heap);

    ensure_invariant();
  }

  /**
   * \brief Returns whether or not the priority queue contains an element with the specified ID.
   *
   * \param[in] id  The ID
   * \return        true, if it does contain such an element, or false otherwise
   */
  bool contains(ID id) const
  {
    return m_dictionary.find(id) != m_dictionary.end();
  }

  /**
   * \brief Returns a reference to the element with the specified ID.
   *
   * param[in] id The ID
   * \pre
   *   - contains(id)
   * return As described
   */
  Element& element(ID id)
  {
    return m_heap[m_dictionary[id]];
  }

  /**
   * \brief Returns whether or not the priority queue is empty.
   *
   * \return true, if is empty, or false if it isn't
   */
  bool empty() const
  {
    return m_dictionary.empty();
  }

  /**
   * \brief Erases the element with the specified ID from the priority queue.
   *
   * \param[in] id The ID
   * \pre
   *   - contains(id)
   * \post
   *   - !contains(id)
   */
  void erase(ID id)
  {
    size_t i = m_dictionary[id];
    m_dictionary.erase(id);
    m_heap[i] = m_heap[m_heap.size()-1];
    if(m_heap[i].id() != id)	// assuming the element we were erasing wasn't the last one in the heap, update the dictionary
    {
      m_dictionary[m_heap[i].id()] = i;
    }
    m_heap.pop_back();
    heapify(i);

    ensure_invariant();
  }

  /**
   * \brief Inserts a new element into the priority queue.
   *
   * \param[in] id   The new element's ID
   * \param[in] key  The new element's key
   * \param[in] data The new element's auxiliary data
   */
  void insert(ID id, const Key& key, const Data& data)
  {
    if(contains(id))
    {
      throw std::runtime_error("An element with the specified ID is already in the priority queue");
    }

    size_t i = m_heap.size();
    m_heap.resize(i+1);
    while(i > 0 && Comp()(key, m_heap[parent(i)].key()))
    {
      size_t p = parent(i);
      m_heap[i] = m_heap[p];
      m_dictionary[m_heap[i].id()] = i;
      i = p;
    }
    m_heap[i] = Element(id, key, data);
    m_dictionary[id] = i;

    ensure_invariant();
  }

  /**
   * \brief Removes the element at the front of the priority queue.
   *
   * \pre
   *   - !empty()
   */
  void pop()
  {
    erase(m_heap[0].id());
    ensure_invariant();
  }

  /**
   * \brief Returns the number of elements in the priority queue.
   */
  size_t size() const
  {
    return m_dictionary.size();
  }

  /**
   * \brief Returns the element at the front of the priority queue.
   *
   * \pre
   *   - !empty()
   * \return As described
   */
  Element top()
  {
    return m_heap[0];
  }

  /**
   * \brief Updates the key of the specified element with a new value.
   *
   * This potentially involves an internal reordering of the priority queue's heap.
   *
   * \param[in] id  The ID of the element whose key is to be updated
   * \param[in] key The new key value
   * \pre
   *   - contains(id)
   */
  void update_key(ID id, const Key& key)
  {
    size_t i = m_dictionary[id];
    update_key_at(i, key);

    ensure_invariant();
  }

  //#################### PRIVATE METHODS ####################
private:
  void ensure_invariant()
  {
    if(m_dictionary.size() != m_heap.size())
    {
      throw std::runtime_error("The operation which just executed invalidated the priority queue");
    }
  }

  void heapify(size_t i)
  {
    bool done = false;
    while(!done)
    {
      size_t L = left(i), R = right(i);
      size_t largest = i;
      if(L < m_heap.size() && Comp()(m_heap[L].key(), m_heap[largest].key()))
        largest = L;
      if(R < m_heap.size() && Comp()(m_heap[R].key(), m_heap[largest].key()))
        largest = R;
      if(largest != i)
      {
        std::swap(m_heap[i], m_heap[largest]);
        m_dictionary[m_heap[i].id()] = i;
        m_dictionary[m_heap[largest].id()] = largest;
        i = largest;
      }
      else done = true;
    }
  }

  inline static size_t left(size_t i)   { return 2*i + 1; }
  inline static size_t parent(size_t i) { return (i+1)/2 - 1; }

  void percolate(size_t i)
  {
    while(i > 0 && Comp()(m_heap[i].key(), m_heap[parent(i)].key()))
    {
      size_t p = parent(i);
      std::swap(m_heap[i], m_heap[p]);
      m_dictionary[m_heap[i].id()] = i;
      m_dictionary[m_heap[p].id()] = p;
      i = p;
    }
  }

  inline static size_t right(size_t i) { return 2*i + 2; }

  void update_key_at(size_t i, const Key& key)
  {
    if(Comp()(key, m_heap[i].key()))
    {
      // The key has increased.
      m_heap[i].m_key = key;
      percolate(i);
    }
    else if(Comp()(m_heap[i].key(), key))
    {
      // The key has decreased.
      m_heap[i].m_key = key;
      heapify(i);
    }
  }


  //#################### SERIALIZATION #################### 
private:
  /**
   * \brief Serializes the priority queue to/from an archive.
   *
   * \param ar      The archive.
   * \param version The file format version number.
   */
  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & m_dictionary;
    ar & m_heap;
  }

  friend class boost::serialization::access;
};

}

#endif
