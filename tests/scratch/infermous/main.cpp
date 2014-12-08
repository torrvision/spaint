//###
#if 0

#include <algorithm>
#include <iostream>
#include <iterator>

#include <infermous/CRF2D.h>
using namespace infermous;

int main()
{
  /*CRF2D<int> crf(10, 10);
  for(CRF2D<int>::iterator it = crf.nodes_begin(), iend = crf.nodes_end(); it != iend; ++it)
  {
    std::cout << it.x() << ' ' << it.y() << '\n';
    CRF2D<int>::Node& n = *it;
  }*/
  typedef int Label;
  CRF2D<int>::ProbabilitiesGrid_Ptr unariesGrid(new CRF2D<int>::ProbabilitiesGrid(5, 5));

  std::map<Label,float> unaries;
  for(size_t i = 0; i < 5; ++i)
    unaries.insert(std::make_pair(i, 0.5f));
  
  for(size_t i = 0; i < 5; ++i)
    for(size_t j = 0; j < 5; ++j)
    {
      (*unariesGrid)(i,j) = unaries;
    }

  CRF2D<int>::print_grid(*unariesGrid);

  CRF2D<int> crf(unariesGrid, 3);

  crf.update();

  // TODO
  return 0;
}

#endif

//###
#if 0

#include <infermous/engines/MeanFieldInferenceEngine.h>
using namespace infermous;

typedef int Label;
typedef CRF2D<Label> CRF;
typedef boost::shared_ptr<CRF> CRF_Ptr;

struct PPC : PairwisePotentialCalculator<Label>
{
  float calculate_potential(const Label& l1, const Label& l2) const
  {
    return l1 == l2 ? 0.0f : 1.0f;
  }
};

int main()
{
  CRF::ProbabilitiesGrid_Ptr unaries(new CRF::ProbabilitiesGrid(5, 5));

  std::map<Label,float> pixelUnaries;
  for(int i = 0; i < 5; ++i)
  {
    pixelUnaries.insert(std::make_pair(i, 0.5f));
  }

  for(size_t i = 0; i < 5; ++i)
    for(size_t j = 0; j < 5; ++j)
    {
      (*unaries)(i,j) = pixelUnaries;
    }

  CRF_Ptr crf(new CRF(unaries, boost::shared_ptr<PPC>(new PPC)));
  crf->output(std::cout);

  MeanFieldInferenceEngine<Label> mfie(crf, MeanFieldUtil::make_circular_neighbour_offsets(3));

  mfie.update_crf();
  crf->output(std::cout);

  mfie.update_crf();
  crf->output(std::cout);

  return 0;
}

#endif

//###
#if 1

#include <algorithm>
#include <functional>
#include <map>
#include <string>

#include <tvgutil/ArgUtil.h>
using tvgutil::ArgUtil;

template <typename K, typename V, typename Pred>
struct SndPred
{
  Pred basePred;

  explicit SndPred(const Pred& basePred_)
  : basePred(basePred_)
  {}

  bool operator()(const std::pair<K,V>& lhs, const std::pair<K,V>& rhs) const
  {
    return basePred(lhs.second, rhs.second);
  }
};

template <typename K, typename V, typename Pred>
const K& argonaut(const std::map<K,V>& m, Pred pred)
{
  return std::min_element(m.begin(), m.end(), SndPred<K,V,Pred>(pred))->first;
}

template <typename K, typename V>
const K& argmax(const std::map<K,V>& m)
{
  return argonaut(m, std::greater<V>());
}

template <typename K, typename V>
const K& argmin(const std::map<K,V>& m)
{
  return argonaut(m, std::less<V>());
}

int main()
{
  std::map<std::string,int> m;
  m["Foo"] = 23;
  m["Bar"] = 9;
  m["Wibble"] = 84;
  m["Wobble"] = 17;

  //std::string lo = std::min_element(m.begin(), m.end(), &snd_pred<std::string,int,std::less<int> >)->first;
  //std::string hi = std::min_element(m.begin(), m.end(), &snd_pred<std::string,int,std::greater<int> >)->first;
  std::string lo = argmin(m);
  std::string hi = argmax(m);

  std::string l = ArgUtil::argmin(m);
  std::string h = ArgUtil::argmax(m);

  return 0;
}

#endif
