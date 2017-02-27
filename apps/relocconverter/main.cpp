/**
 * relocconverter: main.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include <grove/features/interface/RGBDPatchFeatureCalculator.h>
#include <grove/forests/DecisionForestFactory.h>

#include <iostream>

#include <DatasetRGBDInfiniTAM.hpp>

#include <boost/random.hpp>

using namespace grove;

int main(int argc, char *argv[])
{
  static const int nbTrees = 5;
  const float proportionOfDataGivenToLearner = 1.0f;
  const std::string learnerType = "DFBP";
  const bool loadSavedForest = true;
  const int learnFromTree = 0;
  const bool loadFeatures = false;
  const int randomSeed = 42;

  if (argc < 4)
  {
    std::cerr << "Usage: " << argv[0]
        << " \"scoreforest config file\" \"7scenes base path\" \"output forest filename\""
        << std::endl;
    return 1;
  }

  const std::string configFile = argv[1];
  const std::string dataPath = argv[2];
  const std::string outputFile = argv[3];

  boost::shared_ptr<DatasetRGBD7Scenes> m_dataset(
      new DatasetRGBD7Scenes(configFile, dataPath, nbTrees,
          proportionOfDataGivenToLearner, learnerType, loadSavedForest,
          learnFromTree, loadFeatures, randomSeed));
  m_dataset->LoadForest();

  auto scoreForest = DecisionForestFactory<RGBDPatchDescriptor, nbTrees>::make_forest(ITMLib::ITMLibSettings::DEVICE_CPU, *m_dataset->GetForest());

  std::cout << "Saving forest in: " << outputFile << std::endl;
  scoreForest->save_structure_to_file(outputFile);

#if 0
  // Randomly select one prediction per tree
  boost::mt19937 rng(42);

  std::vector<GPUForestPrediction> predictions;
  std::vector<size_t> predictionIndices;
  for (size_t i = 0; i < m_scoreForest->get_nb_trees(); ++i)
  {
    boost::uniform_int<size_t> dist(0,
        m_scoreForest->get_nb_leaves_in_tree(i) - 1);

    size_t predictionIdx = 0;
    GPUForestPrediction p;
    p.nbModes = 0;
    while (p.nbModes < 5)
    {
      predictionIdx = dist(rng);
      std::cout << "Selecting prediction " << predictionIdx << " for tree " << i
      << '\n';
      p = m_scoreForest->get_prediction(i, predictionIdx);
    }

    predictions.push_back(p);
    predictionIndices.push_back(predictionIdx);
  }

  // For each prediction print centroids, covariances, nbInliers
  for (size_t predictionIdx = 0; predictionIdx < m_scoreForest->get_nb_trees();
      ++predictionIdx)
  {
    const GPUForestPrediction &p = predictions[predictionIdx];
    std::cout << p.nbModes << ' ' << predictionIndices[predictionIdx] << '\n';
    for(int modeIdx = 0; modeIdx < p.nbModes; ++modeIdx)
    {
      const GPUForestMode &m = p.modes[modeIdx];
      std::cout << m.nbInliers << ' ' << m.position.x << ' ' << m.position.y << ' ' << m.position.z << ' ';

      // Invet and transpose the covariance to print it in row-major
      Matrix3f posCovariance;
      m.positionInvCovariance.inv(posCovariance);
      posCovariance = posCovariance.t();

      for(int i = 0; i < 9; ++i)
      std::cout << posCovariance.m[i] << ' ';
      std::cout << '\n';
    }
    std::cout << '\n';
  }

#endif

  return 0;
}
