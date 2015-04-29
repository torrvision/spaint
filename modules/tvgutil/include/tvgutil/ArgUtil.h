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
 * \brief This class contains utility functions for performing argmax and argmin operations.
 */
class ArgUtil
{
  //#################### PREDICATES ####################
private:
  /**
   * \brief An instance of an instantiation of this predicate can be used to perform comparisons on the second elements of pairs.
   */
  template <typename T1, typename T2, typename BasePred>
  struct SndPred
  {
    bool operator()(const std::pair<T1,T2>& lhs, const std::pair<T1,T2>& rhs) const
    {
      BasePred basePred;
      return basePred(lhs.second, rhs.second);
    }
  };

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Calculates argmax_k m[k].
   *
   * This function finds a key in the map whose corresponding value is largest (using std::greater).
   * If there are several keys with the largest value, one of them is returned deterministically.
   *
   * \param m The map over which to perform the argmax.
   * \return  A key in the map with the largest corresponding value.
   */
  template <typename K, typename V>
  static const K& argmax(const std::map<K,V>& m)
  {
    return std::min_element(m.begin(), m.end(), SndPred<K,V,std::greater<V> >())->first;
  }

  /**
   * \brief Calculates argmax_k m[k].
   *
   * This function finds the index in the vector whose corresponding value is largest (using std::greater).
   * If there are several keys with the largest value, of them is returned deterministically.
   *
   * \param v The vector over which to perform the argmax.
   * \return  An index in the vector with the largest corresponding value.
   */
  template <typename T>
  static size_t argmax(const std::vector<T>& v)
  {
    return std::distance(v.begin(), std::min_element(v.begin(), v.end(), std::greater<T>()));
  }

  /**
   * \brief Calculates argmin_k m[k].
   *
   * This function finds a key in the map whose corresponding value is smallest (using std::less).
   * If there are several keys with the smallest value, one of them is returned deterministically.
   *
   * \param m The map over which to perform the argmin.
   * \return  A key in the map with the smallest corresponding value.
   */
  template <typename K, typename V>
  static const K& argmin(const std::map<K,V>& m)
  {
    return std::min_element(m.begin(), m.end(), SndPred<K,V,std::less<V> >())->first;
  }

  /**
   * \brief Calculates argmin_k m[k].
   *
   * This function finds the index in the vector whose corresponding value is smallest (using std::less).
   * If there are several keys with the smallest value, one of them is returned deterministically.
   *
   * \param v The vector over which to perform the argmin.
   * \return  An index in the vector with the smallest corresponding value.
   */
  template <typename T>
  static size_t argmin(const std::vector<T>& v)
  {
    return std::distance(v.begin(), std::min_element(v.begin(), v.end(), std::less<T>()));
  }
};

}

#endif
