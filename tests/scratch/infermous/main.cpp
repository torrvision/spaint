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
  ProbabilitiesGrid_Ptr<Label> unaries(new ProbabilitiesGrid<Label>(5, 5));

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
#if 0

#include <infermous/engines/MeanFieldInferenceEngine.h>
using namespace infermous;

#include <tvgutil/numbers/RandomNumberGenerator.h>
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
  ProbabilitiesGrid_Ptr<Label> unaries(new ProbabilitiesGrid<Label>(10, 10));

  for(size_t i = 0; i < 10; ++i)
    for(size_t j = 0; j < 10; ++j)
    {
      if( i >= 3 && i < 7 && j >= 3 && j < 7)
      {
        (*unaries)(i,j)[FG] = 0.9f;
      }
      else
      {
        (*unaries)(i,j)[FG] = 0.1f;
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

//###
#if 1

#include <infermous/engines/MeanFieldInferenceEngine.h>
using namespace infermous;

#include <tvgutil/numbers/RandomNumberGenerator.h>
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
  ProbabilitiesGrid_Ptr<Label> unaries(new ProbabilitiesGrid<Label>(10, 10));

  const int xSize = 10;
  const int ySize = 10;
  const int numberToFlip = 10;

  for(size_t y = 0; y < ySize; ++y)
    for(size_t x = 0; x < xSize; ++x)
    {
      if( x >= 3 && x < 7 && y >= 3 && y < 7)
      {
        (*unaries)(y,x)[FG] = 0.9f;
      }
      else
      {
        (*unaries)(y,x)[FG] = 0.1f;
      }

      (*unaries)(y,x)[BG] = 1.0f - (*unaries)(y,x)[FG];
    }

  tvgutil::RandomNumberGenerator rng(1234);
  for(size_t i = 0; i < numberToFlip; ++i)
  {
    int x = rng.generate_int_from_uniform(0, xSize - 1);
    int y = rng.generate_int_from_uniform(0, ySize -1);
    std::swap((*unaries)(y,x)[FG], (*unaries)(y,x)[BG]);
  }

  CRF2D_Ptr<Label> crf(new CRF2D<Label>(unaries, boost::shared_ptr<PPC>(new PPC)));
  std::cout << crf->predict_labels() << '\n';
  std::cout << *crf << '\n';

  MeanFieldInferenceEngine<Label> mfie(crf, CRFUtil::make_square_neighbour_offsets(1));

  mfie.update_crf(10);
  std::cout << *crf << '\n';
  std::cout << crf->predict_labels() << '\n';

  return 0;
}

#endif
