/**
 * spaint: SpaintInteractor.cpp
 */

#include "interaction/SpaintInteractor.h"

#include "interaction/cpu/ImageProcessing_CPU.h"
#ifdef WITH_CUDA
#include "interaction/cuda/ImageProcessing_CUDA.h"
#endif
#ifdef WITH_OPENCV
#include "interaction/OCVdebugger.h"
#endif

namespace spaint {

//#################### CONSTRUCTOR #################### 

SpaintInteractor::SpaintInteractor(const SpaintModel_Ptr model)
:m_model(model)
{
#ifdef WITH_CUDA
  const bool allocateCPU = true;
  const bool allocateCUDA = true;
  const bool metalCompatible = false;
  m_diffRawRaycast.reset(new ITMFloatImage(model->get_depth_image_size(), allocateCPU, allocateCUDA, metalCompatible));
  m_depthRaycastResult.reset(new ITMFloatImage(model->get_depth_image_size(), allocateCPU, allocateCUDA, metalCompatible));
  m_imageProcessor.reset(new ImageProcessing_CUDA);
#else
  const bool allocateCPU = true;
  const bool allocateCUDA = false;
  const bool metalCompatible = false;
  m_diffRawRaycast.reset(new ITMFloatImage(model->get_depth_image_size(), allocateCPU, allocateCUDA, metalCompatible));
  m_depthRaycastResult.reset(new ITMFloatImage(model->get_depth_image_size(), allocateCPU, allocateCUDA, metalCompatible));
  m_imageProcessor.reset(new ImageProcessing_CPU);
#endif
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void SpaintInteractor::touch(SpaintRaycaster::RenderState_Ptr renderState, const rigging::SimpleCamera_Ptr& camera, const SpaintRaycaster_Ptr& raycaster)
{
  // Get the voxel size which is in meters. This is used to convert from voxel coordinates to meters.
  float voxelSize = m_model->get_settings()->sceneParams.voxelSize;
  
  // Now calculate the depth raycast from the current scene, this is in meters.
  raycaster->generate_depth_raycast(m_depthRaycastResult, renderState, camera, voxelSize);

  // Compate the raw depth from the camera (Meters) to the depth raycast.
  m_imageProcessor->absolute_difference_calculator(m_diffRawRaycast.get(), m_model->get_view()->depth, m_depthRaycastResult.get());
  //debugImage(m_depthRaycastResult.get());
  //debugImage(m_model->get_view()->depth);
  //debugCompareImages(m_depthRaycastResult.get(), m_model->get_view()->depth);
  //debugImage(m_diffRawRaycast.get());

  //debugImage(m_diffRawRaycast.get());
#ifndef WITH_CUDA
#ifdef WITH_OPENCV
  OCVdebugger::display_image_scale_to_range(m_model->get_view()->depth, "Current Raw Depth From Camera Millimeters");
  OCVdebugger::display_image_scale_to_range(m_depthRaycastResult.get(), "Current Depth Raycast Millimeters");
  OCVdebugger::display_image_and_scale(m_diffRawRaycast.get(), 1000.0f, "Difference between the Camera Depth and the Raycasted Depth");
#endif
#endif
}

void SpaintInteractor::debugCompareImages(ITMFloatImage *image1, ITMFloatImage *image2)
{
  const Vector2i& imgSize1 = image1->noDims;
  float *data1 = image1->GetData(MEMORYDEVICE_CPU);
  float *data2 = image2->GetData(MEMORYDEVICE_CPU);
  int elements = imgSize1.x*imgSize1.y;
  for(int i = 0, iend = elements; i < iend; i+=4)
  {
    if((data1[i] > 0) && (data2[i] > 0))
    std::cout << data1[i] << " - " << data2[i] << " = " << data1[i] - data2[i] << "\n";
  }
  std::cout << "\n\n";
}

void SpaintInteractor::debugImage(ITMFloatImage *image)
{
  const Vector2i& imgSize = image->noDims;
  float *data = image->GetData(MEMORYDEVICE_CPU);
  int elements = imgSize.x*imgSize.y; 
  for(int i = 0, iend = elements; i < iend; i+=4)
  {
    std::cout << data[i] << " ";
    if(i % imgSize.x == 0) std::cout << "\n";
  }
  std::cout << "\n\n";

}

}
