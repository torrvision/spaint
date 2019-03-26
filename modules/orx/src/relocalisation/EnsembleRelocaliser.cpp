/**
 * orx: EnsembleRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2019. All rights reserved.
 */

#include "relocalisation/EnsembleRelocaliser.h"

namespace orx {

//#################### CONSTRUCTORS ####################

EnsembleRelocaliser::EnsembleRelocaliser(const std::vector<Relocaliser_Ptr>& innerRelocalisers)
: m_innerRelocalisers(innerRelocalisers)
{
  // Check that the ensemble contains at least one relocaliser.
  if(innerRelocalisers.empty())
  {
    throw std::runtime_error("Error: Cannot create an empty ensemble relocaliser");
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void EnsembleRelocaliser::finish_training()
{
  for(size_t i = 0, size = m_innerRelocalisers.size(); i < size; ++i)
  {
    m_innerRelocalisers[i]->finish_training();
  }
}

ORUChar4Image_CPtr EnsembleRelocaliser::get_visualisation_image(const std::string& key) const
{
  // FIXME: Returning the image from the first relocaliser will do for now, but longer-term we should do this properly.
  return m_innerRelocalisers[0]->get_visualisation_image(key);
}

void EnsembleRelocaliser::load_from_disk(const std::string& inputFolder)
{
  // TODO: Not yet supported.
  throw std::runtime_error("Error: Cannot yet load an ensemble relocaliser from disk");
}

std::vector<Relocaliser::Result>
EnsembleRelocaliser::relocalise(const ORUChar4Image *colourImage, const ORFloatImage *depthImage, const Vector4f& depthIntrinsics) const
{
  std::vector<Result> combinedRelocalisationResults;

  // Try to relocalise with each of the inner relocalisers in turn, and aggregate the results.
  for(size_t i = 0, size = m_innerRelocalisers.size(); i < size; ++i)
  {
    std::vector<Result> relocalisationResults = m_innerRelocalisers[i]->relocalise(colourImage, depthImage, depthIntrinsics);
    std::copy(relocalisationResults.begin(), relocalisationResults.end(), std::back_inserter(combinedRelocalisationResults));
  }

  // Sort the results in ascending order of score, and return them.
  // FIXME: This assumes that the scores produced by different relocalisers are comparable, which may not necessarily be the case.
  std::sort(combinedRelocalisationResults.begin(), combinedRelocalisationResults.end(), &compare_results);

  return combinedRelocalisationResults;
}

void EnsembleRelocaliser::reset()
{
  for(size_t i = 0, size = m_innerRelocalisers.size(); i < size; ++i)
  {
    m_innerRelocalisers[i]->reset();
  }
}

void EnsembleRelocaliser::save_to_disk(const std::string& outputFolder) const
{
  // TODO: Not yet supported.
  throw std::runtime_error("Error: Cannot yet save an ensemble relocaliser to disk");
}

void EnsembleRelocaliser::train(const ORUChar4Image *colourImage, const ORFloatImage *depthImage,
                               const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose)
{
  for(size_t i = 0, size = m_innerRelocalisers.size(); i < size; ++i)
  {
    m_innerRelocalisers[i]->train(colourImage, depthImage, depthIntrinsics, cameraPose);
  }
}

void EnsembleRelocaliser::update()
{
  for(size_t i = 0, size = m_innerRelocalisers.size(); i < size; ++i)
  {
    m_innerRelocalisers[i]->update();
  }
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

bool EnsembleRelocaliser::compare_results(const Result& lhs, const Result& rhs)
{
  return lhs.score < rhs.score;
}

}
