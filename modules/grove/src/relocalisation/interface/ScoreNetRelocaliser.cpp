/**
 * grove: ScoreNetRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "relocalisation/interface/ScoreNetRelocaliser.h"
using namespace ORUtils;

#include <orx/base/MemoryBlockFactory.h>
using namespace orx;

#include <tvgutil/filesystem/PathFinder.h>
using namespace tvgutil;

#define DEBUGGING 0

namespace grove {

//#################### CONSTRUCTORS ####################

ScoreNetRelocaliser::ScoreNetRelocaliser(const tvgutil::SettingsContainer_CPtr& settings, const std::string& settingsNamespace, DeviceType deviceType)
: ScoreRelocaliser(settings, settingsNamespace, deviceType)
{
  // Determine the top-level parameters for the relocaliser.
  m_netType = m_settings->get_first_value<std::string>(settingsNamespace + "netType", "dsac");
  m_reuseRandomWhenFull = m_settings->get_first_value<bool>(settingsNamespace + "reuseRandomWhenFull", false);
  m_useBucketPredictions = m_settings->get_first_value<bool>(settingsNamespace + "useBucketPredictions", true);

  // Determine the bucketing parameters for the relocaliser.
  m_bucketSizeCm = m_settings->get_first_value<int>(settingsNamespace + "bucketSizeCm", 10);
  m_reservoirCount = m_settings->get_first_value<unsigned int>(settingsNamespace + "reservoirCount", 40000);
  m_sceneSizeCm = m_settings->get_first_value<int>(settingsNamespace + "sceneSizeCm", 10000);

  // Allocate the internal images.
  MemoryBlockFactory& mbf = MemoryBlockFactory::instance();
  m_bucketIndicesImage = mbf.make_image<BucketIndices>();

  // Load the SCoRe network from disk.
  const std::string modelFilename = m_settings->get_first_value<std::string>(settingsNamespace + "modelFilename", (find_subdir_from_executable("resources") / "DefaultScoreNet.pt").string());
  m_scoreNet = torch::jit::load(modelFilename);
  if(deviceType == DEVICE_CUDA) m_scoreNet->to(torch::kCUDA);

  // Allocate a memory block to hold the output of the SCoRe network.
  m_scoreNetOutput = MemoryBlockFactory::instance().make_block<float>();

  // Set the step for the feature calculator to ensure that the keypoint/descriptor images are the same size as the network output.
  m_featureCalculator->set_feature_step(8);

  // Set up the example clusterer and the relocaliser's internal state.
  reset();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ScoreNetRelocaliser::reset()
{
  ScoreRelocaliser::reset();
  m_bucketRemapper.clear();
  m_rng.reset(new RandomNumberGenerator(m_rngSeed));
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void ScoreNetRelocaliser::make_predictions(const ORUChar4Image *colourImage) const
{
  if(m_useBucketPredictions)
  {
    // Find the buckets (example reservoirs) corresponding to the keypoints.
    find_reservoirs(colourImage, false);

    // Copy the clusters for each keypoint across to the SCoRe predictions image.
    set_bucket_predictions_for_keypoints(m_bucketIndicesImage, m_predictionsImage);
  }
  else
  {
    // Run the network on the colour image to predict a world space point for each keypoint.
    run_net(colourImage);

    // For each keypoint, copy its corresponding world space point into a single cluster for the keypoint in the SCoRe predictions image.
    set_net_predictions_for_keypoints(m_keypointsImage, m_scoreNetOutput, m_predictionsImage);
  }
}

void ScoreNetRelocaliser::train_sub(const ORUChar4Image *colourImage, const ORFloatImage *depthImage,
                                    const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose)
{
  // Extract keypoints from the RGB-D image and compute descriptors for them.
  // FIXME: We don't need to compute the descriptors when we're using the net (we should allow keypoints to be computed without also computing the descriptors).
  const Matrix4f invCameraPose = cameraPose.GetInvM();
  m_featureCalculator->compute_keypoints_and_features(colourImage, depthImage, invCameraPose, depthIntrinsics, m_keypointsImage.get(), m_descriptorsImage.get());

  // Find the reservoirs to which we should add the keypoints, allocating new reservoirs if necessary.
  find_reservoirs(colourImage, true);

  // Add the keypoints to the relevant reservoirs.
  m_relocaliserState->exampleReservoirs->add_examples(m_keypointsImage, m_bucketIndicesImage);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void ScoreNetRelocaliser::find_reservoirs(const ORUChar4Image *colourImage, bool allowAllocation) const
{
  // Run the network on the colour image to predict a world space point for each keypoint.
  run_net(colourImage);

  // Ensure that the bucket indices image is the same size as the keypoints image.
  const Vector2i imgSize = m_keypointsImage->noDims;
  m_bucketIndicesImage->ChangeDims(imgSize);

  // Compute a bucket index for each keypoint.
  const float *scoreNetOutputPtr = m_scoreNetOutput->GetData(MEMORYDEVICE_CPU);
  const int planeOffset = imgSize.x * imgSize.y;
  BucketIndices *bucketIndices = m_bucketIndicesImage->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int y = 0; y < imgSize.y; ++y)
  {
    for(int x = 0; x < imgSize.x; ++x)
    {
      // Construct the world space point that the network predicted for this keypoint.
      const int pixelOffset = y * imgSize.x + x;
      const Vector3f pos = Vector3f(
        scoreNetOutputPtr[pixelOffset],
        scoreNetOutputPtr[planeOffset + pixelOffset],
        scoreNetOutputPtr[2 * planeOffset + pixelOffset]
      );

      // Use it to compute a bucket index (corresponding to a cubic cell in a grid overlaid on the training scene).
      const int sceneSizeBuckets = m_sceneSizeCm / m_bucketSizeCm;
      const int halfSceneSizeBuckets = sceneSizeBuckets / 2;

      const int bucketX = static_cast<int>(CLAMP(ROUND(pos.x * 100 / m_bucketSizeCm + halfSceneSizeBuckets), 0, sceneSizeBuckets - 1));
      const int bucketY = static_cast<int>(CLAMP(ROUND(pos.y * 100 / m_bucketSizeCm + halfSceneSizeBuckets), 0, sceneSizeBuckets - 1));
      const int bucketZ = static_cast<int>(CLAMP(ROUND(pos.z * 100 / m_bucketSizeCm + halfSceneSizeBuckets), 0, sceneSizeBuckets - 1));
      const int bucketIndex = bucketZ * sceneSizeBuckets * sceneSizeBuckets + bucketY * sceneSizeBuckets + bucketX;

      // Remap the bucket index to one of the example reservoirs. Start by looking to see whether there is already
      // a mapping from the bucket index to a reservoir. If not, and there are reservoirs spare, allocate the first
      // available one. If there aren't any reservoirs spare, pick one to reuse.
      int remappedBucketIndex = 0;

    #ifdef WITH_OPENMP
      #pragma omp critical
    #endif
      {
        std::map<int,int>::const_iterator it = m_bucketRemapper.find(bucketIndex);

        if(it != m_bucketRemapper.end())
        {
          remappedBucketIndex = it->second;
        }
        else if(allowAllocation)
        {
          if(m_bucketRemapper.size() < m_reservoirCount)
          {
            // Use the next available reservoir.
            remappedBucketIndex = static_cast<int>(m_bucketRemapper.size());
          }
          else if(m_reuseRandomWhenFull)
          {
            // Pick the reservoir to reuse randomly.
            remappedBucketIndex = m_rng->generate_int_from_uniform(0, static_cast<int>(m_reservoirCount - 1));
          }
          else
          {
            // Pick the reservoir to reuse deterministically.
            remappedBucketIndex = static_cast<int>(m_bucketRemapper.size() % m_reservoirCount);
          }

          m_bucketRemapper.insert(std::make_pair(bucketIndex, remappedBucketIndex));
        }
      }

      // Store the remapped bucket index in the bucket indices image.
      bucketIndices[pixelOffset][0] = remappedBucketIndex;
    }
  }

  // Copy the bucket indices image across to the GPU (if we're using it).
  m_bucketIndicesImage->UpdateDeviceFromHost();

#if DEBUGGING
  std::cout << "Buckets Used: " << m_bucketRemapper.size() << std::endl;
#endif
}

void ScoreNetRelocaliser::run_net(const ORUChar4Image *colourImage) const
{
  // Copy the colour image across to the CPU if necessary.
  colourImage->UpdateHostFromDevice();

  // Calculate the normalisation constants for the values in the colour image, based on the type of network being used.
  float normalisationOffsetR, normalisationOffsetG, normalisationOffsetB,
        normalisationFactorR, normalisationFactorG, normalisationFactorB,
        normalisationFactorCommon;

  if(m_netType == "dsac")
  {
    normalisationOffsetR = normalisationOffsetG = normalisationOffsetB = 127.0f;
    normalisationFactorR = normalisationFactorG = normalisationFactorB = 128.0f;
    normalisationFactorCommon = 1.0f;
  }
  else if(m_netType == "vgg")
  {
    normalisationOffsetR = 0.485f;
    normalisationOffsetG = 0.456f;
    normalisationOffsetB = 0.406f;
    normalisationFactorR = 0.229f;
    normalisationFactorG = 0.224f;
    normalisationFactorB = 0.225f;
    normalisationFactorCommon = 255.0f;
  }
  else throw std::runtime_error("Error: Unknown network type '" + m_netType + "'");

  // Copy the pixels of the colour image into an input tensor that can be passed to the network, normalising the values in the process.
  const Vector4u *in = colourImage->GetData(MEMORYDEVICE_CPU);
  const int width = colourImage->noDims.x, height = colourImage->noDims.y;
  torch::Tensor mid = torch::zeros({1,3,height,width});
  auto midAcc = mid.accessor<float,4>();
  for(int y = 0; y < height; ++y)
  {
    for(int x = 0; x < width; ++x)
    {
      const Vector4u& p = in[y * colourImage->noDims.x + x];
      midAcc[0][0][y][x] = ((static_cast<float>(p.r) / normalisationFactorCommon) - normalisationOffsetR) / normalisationFactorR;
      midAcc[0][1][y][x] = ((static_cast<float>(p.g) / normalisationFactorCommon) - normalisationOffsetG) / normalisationFactorG;
      midAcc[0][2][y][x] = ((static_cast<float>(p.b) / normalisationFactorCommon) - normalisationOffsetB) / normalisationFactorB;
    }
  }

  mid = mid.to(torch::kCUDA);

  // Run the network on the input tensor to produce an output tensor.
  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(mid);
  auto out = m_scoreNet->forward(inputs).toTensor();
  out = out.to(torch::kCPU);

  // Copy the output tensor into a memory block so that it can be used later.
  const float *outData = out.data<float>();
  const int outputLen = 3 * m_keypointsImage->noDims.y * m_keypointsImage->noDims.x;
  m_scoreNetOutput->Resize(outputLen);
  std::copy(outData, outData + outputLen, m_scoreNetOutput->GetData(MEMORYDEVICE_CPU));
  m_scoreNetOutput->UpdateDeviceFromHost();
}

}
