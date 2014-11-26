/**
 * tvgutil: LimitedContainer.h
 */

#ifndef H_TVGUTIL_LIMITEDCONTAINER
#define H_TVGUTIL_LIMITEDCONTAINER

#include <iterator>
#include <ostream>
#include <stdexcept>
#include <utility>

namespace tvgutil {

/**
 * \brief An instance of an instantiation of this class template can be used to combine a container that will be
 *        output to a stream with a limit on the number of elements that should be explicitly output.
 *
 * Outputting a limited container to a stream can be used to convey a sense of the contents of the original container
 * without showing everything it contains.
 */
template <typename Cont>
class LimitedContainer
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The container we want to output. */
  const Cont& m_cont;

  /** The maximum number of elements that should be output to the stream. */
  size_t m_limit;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a "limited" container that can be used to output a container to a stream without showing all of its elements.
   *
   * \param cont  The container we want to output.
   * \param limit The maximum number of elements that should be output to the stream.
   */
  LimitedContainer(const Cont& cont, size_t limit)
  : m_cont(cont), m_limit(limit)
  {
    if(limit <= 1) throw std::runtime_error("Container limits must be > 1");
  }

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Outputs the specified element to the specified stream.
   *
   * \param os  The stream.
   * \param rhs The element.
   * \return    The stream.
   */
  template <typename T>
  static std::ostream& output_element(std::ostream& os, const T& rhs)
  {
    os << rhs;
    return os;
  }

  /**
   * \brief Outputs the specified pair to the specified stream.
   *
   * \param os  The stream.
   * \param rhs The element.
   * \return    The stream.
   */
  template <typename K, typename V>
  static std::ostream& output_element(std::ostream& os, const std::pair<K,V>& rhs)
  {
    os << '(' << rhs.first << ',' << rhs.second << ')';
    return os;
  }

  /**
   * \brief Outputs a limited number of elements in the specified range to the specified stream.
   *
   * \param os      The stream.
   * \param begin   An iterator pointing to the start of the range.
   * \param end     An iterator pointing to the end of the range.
   * \param rbegin  An iterator pointing to one before the end of the range.
   * \param limit   The limit value.
   * \return        The stream.
   */
  template <typename Iter, typename RIter>
  static std::ostream& output_limited(std::ostream& os, const Iter& begin, const Iter& end, const RIter& rbegin, size_t limit)
  {
    size_t size = std::distance(begin, end);
    size_t startElts = size <= limit ? size : limit - 1;
    bool printLast = size > limit;

    os << "[ ";

    size_t i = 0;
    for(Iter it = begin, iend = end; it != iend; ++it, ++i)
    {
      if(i == startElts) break;
      output_element(os, *it);
      os << ' ';
    }

    if(printLast)
    {
      os << "... ";
      output_element(os, *rbegin);
      os << ' ';
    }

    os << ']';
    return os;
  }

  //#################### STREAM OPERATORS ####################
public:
  /**
   * \brief Outputs a limited number of elements from the container to the specified stream.
   *
   * If the container contains further elements, these are output using ... (an ellipsis).
   *
   * Example: With a limit of 3, (e1,e2,e3,e4,e5) would be output as [ e1 e2 ... e5 ].
   *
   * \param os  The stream to which to output the container.
   * \param rhs The container to output.
   * \return    The stream.
   */
  friend std::ostream& operator<<(std::ostream& os, const LimitedContainer& rhs)
  {
    return output_limited(os, rhs.m_cont.begin(), rhs.m_cont.end(), rhs.m_cont.rbegin(), rhs.m_limit);
  }
};

//#################### GLOBAL FUNCTIONS ####################

/**
 * \brief Constructs a "limited" container from a container and a limit value.
 *
 * \param cont  The container.
 * \param limit The limit value.
 * \return      The "limited" container.
 */
template <typename Cont>
LimitedContainer<Cont> make_limited_container(const Cont& cont, size_t limit)
{
  return LimitedContainer<Cont>(cont, limit);
}

}

#endif
