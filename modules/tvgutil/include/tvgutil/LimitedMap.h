/**
 * tvgutil: LimitedMap.h
 */

#ifndef H_TVGUTIL_LIMITEDMAP
#define H_TVGUTIL_LIMITEDMAP

#include <map>
#include <ostream>

namespace tvgutil { 
  
/**
 * \brief The LimitedMap class represents a map of values to be printed to the screen with a limit that defines the maximum output size
 */
template <typename K, typename V>
class LimitedMap
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The maximum number of elemnts to print. */
  size_t m_limit;
  
  /** The map structure linking key K and value V */
  const std::map<K,V>& m_m;
  
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an map with a limit value (The maximum number of pairs to print).
   *
   * \param m_limit  The maximum number of elemnts to print.
   * \param rng      The map structure linking key K and value V.
   */
  LimitedMap(const std::map<K,V>& m, size_t limit)
  : m_limit(limit), m_m(m)
  {
    assert(limit > 1);
  }
  
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Prints the pairs in the map from [(first_pair) (limit_pair - 1) ... (last_pair)] to the screen.
   *
   * \param rhs  The LimitedMap to be printed.
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

/**
  * \brief A function which constructs a LimitedMap from a std::map and limit value.
  *
  * \param m  The templated map!.
  * \param limit  The limit!. 
  * \return       The constucted LimitedMap!.
  */
template <typename K, typename V>
LimitedMap<K,V> make_limited_map(const std::map<K,V>& m, size_t limit)
{
  return LimitedMap<K,V>(m, limit);
}

}

#endif
