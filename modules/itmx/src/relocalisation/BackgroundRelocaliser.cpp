/**
 * itmx: BackgroundRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include <iostream>

#include "relocalisation/BackgroundRelocaliser.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

BackgroundRelocaliser::BackgroundRelocaliser(const Relocaliser_Ptr& relocaliser, int relocalisationDevice)
: m_relocalisationDevice(relocalisationDevice), m_relocaliser(relocaliser)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

boost::optional<Relocaliser::Result> BackgroundRelocaliser::relocalise(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage, const Vector4f& depthIntrinsics) const
{
  colourImage->UpdateHostFromDevice();
  depthImage->UpdateHostFromDevice();
  to_relocalisation_gpu();
  copy_images(colourImage, depthImage);
  boost::optional<Relocaliser::Result> result = m_relocaliser->relocalise(m_colourImage.get(), m_depthImage.get(), depthIntrinsics);
  to_old_gpu();
  return result;
}

void BackgroundRelocaliser::reset()
{
  to_relocalisation_gpu();
  m_relocaliser->reset();
  to_old_gpu();
}

void BackgroundRelocaliser::train(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage,
                                  const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose)
{
  colourImage->UpdateHostFromDevice();
  depthImage->UpdateHostFromDevice();
  to_relocalisation_gpu();
  copy_images(colourImage, depthImage);
  m_relocaliser->train(m_colourImage.get(), m_depthImage.get(), depthIntrinsics, cameraPose);
  to_old_gpu();
}

void BackgroundRelocaliser::update()
{
  to_relocalisation_gpu();
  m_relocaliser->update();
  to_old_gpu();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void BackgroundRelocaliser::copy_images(const ITMUChar4Image *colourImage, const ITMFloatImage *depthImage) const
{
  if(!m_colourImage) m_colourImage.reset(new ITMUChar4Image(colourImage->noDims, true, true));
  if(!m_depthImage) m_depthImage.reset(new ITMFloatImage(depthImage->noDims, true, true));

  m_colourImage->ChangeDims(colourImage->noDims);
  m_depthImage->ChangeDims(depthImage->noDims);

  m_colourImage->SetFrom(colourImage, ITMUChar4Image::CPU_TO_CPU);
  m_depthImage->SetFrom(depthImage, ITMFloatImage::CPU_TO_CPU);

  m_colourImage->UpdateDeviceFromHost();
  m_depthImage->UpdateDeviceFromHost();
}

void BackgroundRelocaliser::to_old_gpu() const
{
  //ORcudaSafeCall(cudaDeviceSynchronize()); // TEMPORARY
  ORcudaSafeCall(cudaSetDevice(m_oldDevice));

  /*int temp;
  cudaGetDevice(&temp);
  std::cout << "Now on device " << temp << '\n';*/
}

void BackgroundRelocaliser::to_relocalisation_gpu() const
{
  ORcudaSafeCall(cudaGetDevice(&m_oldDevice));
  //ORcudaSafeCall(cudaDeviceSynchronize()); // TEMPORARY
  ORcudaSafeCall(cudaSetDevice(m_relocalisationDevice));

  /*int temp;
  cudaGetDevice(&temp);
  std::cout << "Now on device " << temp << '\n';*/
}

}
