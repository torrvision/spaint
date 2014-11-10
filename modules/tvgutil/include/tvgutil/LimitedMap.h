/**
 * tvgutil: LimitedMap.h
 */

#ifndef H_TVGUTIL_LIMITEDMAP
#define H_TVGUTIL_LIMITEDMAP

#include <cassert>
#include <map>
#include <ostream>

namespace tvgutil { 
  
/**
 * \brief An instance of an instantiation of this class template can be used to combine a std::map that will be
 *        output to a stream with a limit on the number of elements that should be explicitly output.
 *
 * Outputting a limited map to a stream can be used to convey a sense of the contents of the original map without
 * showing everything it contains.
 */
template <typename K, typename V>
class LimitedMap
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The maximum number of map elements that should be output to the stream. */
  size_t m_limit;
  
  /** The map we want to output. */
  const std::map<K,V>& m_m;
  
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a "limited" map that can be used to output a map to a stream without showing all of its elements.
   *
   * \param m     The map we want to output.
   * \param limit The maximum number of map elements that should be output to the stream.
   */
  LimitedMap(const std::map<K,V>& m, size_t limit)
  : m_limit(limit), m_m(m)
  {
    assert(limit > 1);
  }
  
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Outputs a limited number of pairs from the map to the specified stream. If the map contains further pairs, these are output using ... (an ellipsis).
   *
   * Example: With a limit of 3, (p1,p2,p3,p4,p5) would be output as [ p1 p2 ... p5 ].
   *
   * \param os  The stream to which to output the map.
   * \param rhs The map to output.
   * \return    The stream.
   */
  friend std::ostream& operator<<(std::ostream& os, const LimitedMap& rhs)
  {
    size_t startElts = rhs.m_m.size() <= rhs.m_limit ? rhs.m_m.size() : rhs.m_limit - 1;
    bool printLast = rhs.m_m.size() > rhs.m_limit;

    os << "[ ";

    size_t i = 0;
    for(typename std::map<K,V>::const_iterator it = rhs.m_m.begin(), iend = rhs.m_m.end(); it != iend; ++it, ++i)
    {
      if(i == startElts) break;
      os << '(' << it->first << ',' << it->second << ") ";
    }

    if(printLast)
    {
      std::pair<K,V> last = *rhs.m_m.rbegin();
      os << "... (" << last.first << ',' << last.second << ") ";
    }

    os << ']';
    return os;
  }
};

//#################### GLOBAL FUNCTIONS ####################

/**
 * \brief Constructs a "limited" map from a std::map and a limit value.
 *
 * \param m      The map.
 * \param limit  The limit value.
 * \return       The "limited" map.
 */
template <typename K, typename V>
LimitedMap<K,V> make_limited_map(const std::map<K,V>& m, size_t limit)
{
  return LimitedMap<K,V>(m, limit);
}

}

#endif
