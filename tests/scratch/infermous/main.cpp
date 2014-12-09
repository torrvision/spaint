//###
#if 0

#include <iostream>

#include <infermous/engines/MeanFieldInferenceEngine.h>
using namespace infermous;

typedef int Label;

struct PPC : PairwisePotentialCalculator<Label>
{
  float calculate_potential(const Label& l1, const Label& l2) const
  {
    return l1 == l2 ? 0.0f : 1.0f;
  }
};

int main()
{
  PotentialsGrid_Ptr<Label> unaries(new PotentialsGrid<Label>(5, 5));

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

  CRF2D_Ptr<Label> crf(new CRF2D<Label>(unaries, boost::shared_ptr<PPC>(new PPC)));
  std::cout << *crf << '\n';

  MeanFieldInferenceEngine<Label> mfie(crf, CRFUtil::make_circular_neighbour_offsets(3));

  mfie.update_crf();
  std::cout << *crf << '\n';

  mfie.update_crf();
  std::cout << *crf << '\n';

  std::cout << crf->predict_labels() << '\n';

  return 0;
}

#endif

//###
#if 1

#include <infermous/engines/MeanFieldInferenceEngine.h>
using namespace infermous;

#include <tvgutil/RandomNumberGenerator.h>
using namespace tvgutil;

enum Label
{
  BG,
  FG
};

struct PPC : PairwisePotentialCalculator<Label>
{
  float calculate_potential(const Label& l1, const Label& l2) const
  {
    return l1 == l2 ? 0.0f : 1.0f;
  }
};

int main()
{

  PotentialsGrid_Ptr<Label> unaries(new PotentialsGrid<Label>(10, 10));

  for(size_t i = 0; i < 10; ++i)
    for(size_t j = 0; j < 10; ++j)
    {
      if( i >= 3 && i < 7 && j >= 3 && j < 7)
      {
        (*unaries)(i,j)[FG] = 1.0f;
      }
      else
      {
        (*unaries)(i,j)[FG] = 0.0f;
      }

      (*unaries)(i,j)[BG] = 1.0f - (*unaries)(i,j)[FG];
    }

  CRF2D_Ptr<Label> crf(new CRF2D<Label>(unaries, boost::shared_ptr<PPC>(new PPC)));
  std::cout << crf->predict_labels() << '\n';
  std::cout << *crf << '\n';

  MeanFieldInferenceEngine<Label> mfie(crf, CRFUtil::make_square_neighbour_offsets(1));

  mfie.update_crf(1);
  std::cout << *crf << '\n';
  std::cout << crf->predict_labels() << '\n';

  return 0;
}

#endif
