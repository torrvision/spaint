/**
 * spaint: RGBDUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_RGBDUTIL
#define H_SPAINT_RGBDUTIL

#include <ITMLib/Objects/Camera/ITMRGBDCalib.h>

namespace spaint {

/**
 * \brief This class contains functions that help us to map between the coordinate frames associated with the depth and RGB cameras.
 */
class RGBDUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Calculates a matrix mapping points from 3D depth image coordinates to 3D RGB image coordinates.
   *
   * \param calib The RGBD calibration parameters.
   * \return      A matrix mapping points from 3D depth image coordinates to 3D RGB image coordinates.
   */
  static Matrix4f calculate_depth_to_rgb_matrix_3D(const ITMLib::ITMRGBDCalib& calib);

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Makes a 4x4 calibration matrix from a set of intrinsic camera parameters.
   *
   * See p.141 of Multiple View Geometry in Computer Vision.
   *
   * \param intrinsics  The intrinsic camera parameters.
   * \return            The corresponding 4x4 calibration matrix.
   */
  static Matrix4f make_calibration_matrix(const ITMLib::ITMIntrinsics& intrinsics);
};

}

#endif
