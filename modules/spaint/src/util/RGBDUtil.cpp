/**
 * spaint: RGBDUtil.cpp
 */

#include "util/RGBDUtil.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

Matrix4f RGBDUtil::calculate_depth_to_rgb_matrix_3D(const ITMLib::Objects::ITMRGBDCalib& calib)
{
  // Calculate the transformation from 3D world coordinates (depth) to 3D image coordinates (depth).
  Matrix4f depthCalib = make_calibration_matrix(calib.intrinsics_d);

  // Calculate the transformation from 3D world coordinates (RGB) to 3D image coordinates (RGB).
  Matrix4f rgbCalib = make_calibration_matrix(calib.intrinsics_rgb);

  // Calculate the transformation from 3D image coordinates (depth) to 3D world coordinates (depth).
  Matrix4f invDepthCalib;
  depthCalib.inv(invDepthCalib);

  // Calculate the transformation from 3D image coordinates (depth) to 3D image coordinates (RGB).
  return rgbCalib * calib.trafo_rgb_to_depth.calib_inv * invDepthCalib;
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

Matrix4f RGBDUtil::make_calibration_matrix(const ITMLib::Objects::ITMIntrinsics& intrinsics)
{
  /*
  The indices here are column-first not row-first, thus:

  K = [fx 0  px 0]
      [0  fy py 0]
      [0  0  1  0]
      [0  0  0  1]

  The camera calibration matrix K converts from 3D world coordinates to 3D image coordinates. In particular, given a (homogeneous) 3D point [X,Y,Z,1]
  in 3D world space, it maps it to [fx.X + px.Z, fy.Y + py.Z, Z, 1] in 3D image space. This point can subsequently be projected into 2D by dividing by
  Z to give [fx.X/Z + px, fy.Y/Z + py, 1, 1/Z] and then keeping only the x and y components to obtain a 2D point.

  The inverse transformation from a point [x,y] in 2D image coordinates with known depth z to 3D world coordinates can be accomplished by forming the
  corresponding point p = [xz, yz, z, 1] in 3D image coordinates and then computing K^-1(p).
  */
  Matrix4f K;
  K.setZeros();
  K.m00 = intrinsics.projectionParamsSimple.fx;
  K.m11 = intrinsics.projectionParamsSimple.fy;
  K.m20 = intrinsics.projectionParamsSimple.px;
  K.m21 = intrinsics.projectionParamsSimple.py;
  K.m22 = 1.0f;
  K.m33 = 1.0f;
  return K;
}

}
