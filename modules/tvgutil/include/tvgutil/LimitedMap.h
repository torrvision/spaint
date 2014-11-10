/**
 * tvgutil: LimitedMap.h
 */

#ifndef H_TVGUTIL_LIMITEDMAP
#define H_TVGUTIL_LIMITEDMAP

#include <map>
#include <ostream>

namespace tvgutil { 

template <typename K, typename V>
class LimitedMap
{
private:
  size_t m_limit;

  const std::map<K,V>& m_m;

public:
  LimitedMap(const std::map<K,V>& m, size_t limit)
  : m_limit(limit), m_m(m)
  {
    assert(limit > 1);
  }

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

template <typename K, typename V>
LimitedMap<K,V> make_limited_map(const std::map<K,V>& m, size_t limit)
{
  return LimitedMap<K,V>(m, limit);
}

}

#endif
