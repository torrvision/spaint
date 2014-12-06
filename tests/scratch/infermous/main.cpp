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
#if 1

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
  // TODO: Fill in the initial unaries.

  CRF_Ptr crf(new CRF(unaries, boost::shared_ptr<PPC>(new PPC)));

  MeanFieldInferenceEngine<Label> mfie(crf, 3.0f);
  mfie.update_crf();

  return 0;
}

#endif
