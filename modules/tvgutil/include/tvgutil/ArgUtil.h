/**
 * tvgutil: ArgUtil.h
 */

#ifndef H_TVGUTIL_ARGUTIL
#define H_TVGUTIL_ARGUTIL

#include <algorithm>
#include <functional>
#include <map>

namespace tvgutil {

/**
 * \brief TODO
 */
class ArgUtil
{
  //#################### PREDICATES ####################
private:
  template <typename K, typename V, typename BasePred>
  struct SndPred
  {
    bool operator()(const std::pair<K,V>& lhs, const std::pair<K,V>& rhs) const
    {
      BasePred basePred;
      return basePred(lhs.second, rhs.second);
    }
  };

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  template <typename K, typename V>
  static const K& argmax(const std::map<K,V>& m)
  {
    return std::min_element(m.begin(), m.end(), SndPred<K,V,std::greater<V> >())->first;
  }

  template <typename K, typename V>
  static const K& argmin(const std::map<K,V>& m)
  {
    return std::min_element(m.begin(), m.end(), SndPred<K,V,std::less<V> >())->first;
  }
};

}

#endif
