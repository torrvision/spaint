/**
 * spaint: ColourConversion_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_COLOURCONVERSION_SHARED
#define H_SPAINT_COLOURCONVERSION_SHARED

#include <ITMLib/Objects/Scene/ITMRepresentationAccess.h>

namespace spaint {

/**
 * \brief Converts an RGB colour to greyscale.
 *
 * The conversion formula is from OpenCV, and takes into account the fact that the human
 * eye is more sensitive to green than red or blue. See:
 *
 * http://docs.opencv.org/2.4.11/modules/imgproc/doc/miscellaneous_transformations.html.
 */
_CPU_AND_GPU_CODE_
inline float convert_rgb_to_grey(float r, float g, float b)
{
  return 0.299f * r + 0.587f * g + 0.114f * b;
}

/**
 * \brief A helper function for the RGB to CIELab conversion.
 */
_CPU_AND_GPU_CODE_
inline float rgb_to_lab_f(float t)
{
  return t > 0.008856f ? pow(t, 1.0f / 3.0f) : 7.787f * t + 16.0f / 116.0f;
}

/**
 * \brief Converts an RGB colour to CIELab.
 *
 * \param rgb The RGB colour.
 * \return    The result of converting the colour to CIELab.
 */
_CPU_AND_GPU_CODE_
inline Vector3f convert_rgb_to_lab(const Vector3f& rgb)
{
  // Equivalent Matlab code can be found at: https://www.eecs.berkeley.edu/Research/Projects/CS/vision/bsds/code/Util/RGB2Lab.m
  // See also: http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html
  const float EPSILON = 0.000001f;

  float x = 0.412453f * rgb.r + 0.357580f * rgb.g + 0.180423f * rgb.b;
  float y = 0.212671f * rgb.r + 0.715160f * rgb.g + 0.072169f * rgb.b;
  float z = 0.019334f * rgb.r + 0.119193f * rgb.g + 0.950227f * rgb.b;

  x /= 0.950456f;
  z /= 1.088754f;

  float fx = rgb_to_lab_f(x);
  float fy = rgb_to_lab_f(y);
  float fz = rgb_to_lab_f(z);

  float L = y > 0.008856f ? (116.0f * fy - 16.0f) : (903.3f * y);
  float A = 500.0f * (fx - fy);
  float B = 200.0f * (fy - fz);

  float AplusB = fabs(A + B) + EPSILON;
  A /= AplusB;
  B /= AplusB;

  return Vector3f(L, A, B);
}

/**
 * \brief Converts an RGB colour to YCbCr.
 *
 * \param rgb The RGB colour.
 * \return    The resulting of converting the colour to YCbCr.
 */
_CPU_AND_GPU_CODE_
inline Vector3f convert_rgb_to_ycbcr(const Vector3u& rgb)
{
  // This is adapted from http://www.equasys.de/colorconversion.html.
  return Vector3f(
    CLAMP(0.299f * rgb.r + 0.587f * rgb.g + 0.114f * rgb.b, 0, 255),
    CLAMP(127.5f - 0.169f * rgb.r - 0.331f * rgb.g + 0.5f * rgb.b, 0, 255),
    CLAMP(127.5f + 0.5f * rgb.r - 0.419f * rgb.g - 0.081f * rgb.b, 0, 255)
  );
}

/**
 * \brief Converts a YCbCr colour to RGB.
 *
 * \param ycc The YCbCr colour.
 * \return    The resulting of converting the colour to RGB.
 */
_CPU_AND_GPU_CODE_
inline Vector3u convert_ycbcr_to_rgb(const Vector3f& ycc)
{
  // This is adapted from http://www.equasys.de/colorconversion.html.
  // The RGB -> YCbCr matrix was manually inverted in Matlab.
  float Y = ycc.x;
  float Cbm = ycc.y - 127.5f;
  float Crm = ycc.z - 127.5f;
  return Vector3u(
    (unsigned char)CLAMP(ROUND(Y - 0.0009f * Cbm + 1.4017f * Crm), 0, 255),
    (unsigned char)CLAMP(ROUND(Y - 0.3437f * Cbm - 0.7142f * Crm), 0, 255),
    (unsigned char)CLAMP(ROUND(Y + 1.7722f * Cbm + 0.001f * Crm), 0, 255)
  );
}

}

#endif
