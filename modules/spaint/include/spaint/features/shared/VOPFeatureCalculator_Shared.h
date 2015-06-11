/**
 * spaint: VOPFeatureCalculator_Shared.h
 */

#ifndef H_SPAINT_VOPFEATURECALCULATOR_SHARED
#define H_SPAINT_VOPFEATURECALCULATOR_SHARED

#include <ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h>

#include "../../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief A helper function for the RGB to CIELab conversion.
 */
_CPU_AND_GPU_CODE_
inline float rgb_to_lab_f(float t)
{
  return t > 0.008856f ? pow(t, 1.0f / 3.0f) : 7.787f * t + 16.0f / 116.0f;
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void convert_patch_to_lab(int voxelLocationIndex, size_t featureCount, float *features)
{
  // Convert each RGB colour in the VOP patch of a feature vector to the CIELab colour space.
  for(size_t i = voxelLocationIndex * featureCount, end = i + featureCount - 4; i != end; i += 3)
  {
    // Equivalent Matlab code can be found at: https://www.eecs.berkeley.edu/Research/Projects/CS/vision/bsds/code/Util/RGB2Lab.m
    // See also: http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html
    float r = features[i] / 255.0f;
    float g = features[i+1] / 255.0f;
    float b = features[i+2] / 255.0f;

    float x = 0.412453f * r + 0.357580f * g + 0.180423f * b;
    float y = 0.212671f * r + 0.715160f * g + 0.072169f * b;
    float z = 0.019334f * r + 0.119193f * g + 0.950227f * b;

    x /= 0.950456f;
    z /= 1.088754f;

    float fx = rgb_to_lab_f(x);
    float fy = rgb_to_lab_f(y);
    float fz = rgb_to_lab_f(z);

    float L = y > 0.008856f ? (116.0f * fy - 16.0f) : (903.3f * y);
    float A = 500.0f * (fx - fy);
    float B = 200.0f * (fy - fz);

    //float AplusB = A + B;

    features[i] = L;
    features[i+1] = A/* / AplusB*/;
    features[i+2] = B/* / AplusB*/;
  }
}

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
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void compute_histogram_for_patch(int tid, int patchSize, float *intensityPatch, int binCount, float *histogram)
{
  int indexInPatch = tid % (patchSize * patchSize);

  size_t y = indexInPatch / patchSize;
  size_t x = indexInPatch % patchSize;

  if(x != 0 && y != 0 && x != patchSize - 1 && y != patchSize - 1)
  {
    // Compute the derivatives.
    float xDeriv = intensityPatch[indexInPatch + 1] - intensityPatch[indexInPatch - 1];
    float yDeriv = intensityPatch[indexInPatch + patchSize] - intensityPatch[indexInPatch - patchSize];

    // Compute the magnitude.
    float mag = static_cast<float>(sqrt(xDeriv * xDeriv + yDeriv * yDeriv));

    // Compute the orientation.
    double ori = atan2(yDeriv, xDeriv) + 2 * M_PI;

    // Quantize the orientation and update the histogram.
    int bin = static_cast<int>(binCount * ori / (2 * M_PI)) % binCount;

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
    atomicAdd(&histogram[bin], mag);
    //histogram[bin] += mag; // note: this will need synchronisation!
#else
  #ifdef WITH_OPENMP
    #pragma omp atomic
  #endif
    histogram[bin] += mag; // note: this will need synchronisation!
#endif
  }
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void fill_in_normal_feature(int voxelLocationIndex, const Vector3f *surfaceNormals, size_t featureCount, float *features)
{
  const Vector3f& n = surfaceNormals[voxelLocationIndex];
  float *featuresForVoxel = features + voxelLocationIndex * featureCount;
  size_t offset = featureCount - 4;

  featuresForVoxel[offset] = n.x;
  featuresForVoxel[offset + 1] = n.y;
  featuresForVoxel[offset + 2] = n.z;
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void update_patch_coordinate_system(int tid, int patchArea, int binCount, float *histogram, Vector3f *xAxis, Vector3f *yAxis)
{
  if(tid % patchArea == 0)
  {
    size_t dominantBin;
    double highestBinValue = 0;
    for(size_t binIndex = 0; binIndex < binCount; ++binIndex)
    {
      double binValue = histogram[binIndex];
      if(binValue >= highestBinValue)
      {
        highestBinValue = binValue;
        dominantBin = binIndex;
      }
    }

    float binAngle = static_cast<float>(2 * M_PI) / binCount;
    float dominantOrientation = dominantBin * binAngle;

    float c = cos(dominantOrientation);
    float s = sin(dominantOrientation);

    Vector3f xAxisCopy = *xAxis;
    Vector3f yAxisCopy = *yAxis;

    *xAxis = c * xAxisCopy + s * yAxisCopy;
    *yAxis = c * yAxisCopy - s * xAxisCopy;
  }
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void compute_intensities_for_patch(int tid, const float *features, int featureCount, int patchSize, float *intensityPatch)
{
  int patchArea = patchSize * patchSize;
  int voxelLocationIndex = tid / patchArea;
  int indexInPatch = tid % patchArea;

  const float *rgbPatch = features + voxelLocationIndex * featureCount;
  int pixelOffset = indexInPatch * 3;
  float r = rgbPatch[pixelOffset];
  float g = rgbPatch[pixelOffset + 1];
  float b = rgbPatch[pixelOffset + 2];

  intensityPatch[indexInPatch] = convert_rgb_to_grey(r, g, b);
}

/**
 * \brief Generates a unit vector that is perpendicular to the specified plane normal.
 *
 * The vector generated will be the normalised cross product of the specified plane normal and another vector
 * that is non-parallel to the normal. This non-parallel vector will be the up vector (0,0,1), unless that is
 * parallel to the normal, in which case (1,0,0) will be used instead.
 *
 * \param n The normal of the plane in which we want to generate the unit vector.
 * \return  The unit coplanar vector v as specified, satisfying v.dot(n) == 0.
 */
_CPU_AND_GPU_CODE_
inline Vector3f generate_arbitrary_coplanar_unit_vector(const Vector3f& n)
{
  Vector3f up(0.0f, 0.0f, 1.0f);
  if(fabs(n.x) < 1e-3f && fabs(n.y) < 1e-3f)
  {
    // Special Case: n is too close to the vertical and hence n x up is roughly equal to (0,0,0).
    // Use (1,0,0) instead of up and apply the same method as in the else clause.
    up = Vector3f(1.0f, 0.0f, 0.0f);
    return normalize(cross(n, Vector3f(1.0f, 0.0f, 0.0f)));
  }
  else
  {
    // The normalized cross product of n and up satisfies the requirements of being
    // unit length and perpendicular to n (since we dealt with the special case where
    // n x up is zero, in all other cases it must be non-zero and we can normalize it
    // to give us a unit vector).
    return normalize(cross(n, up));
  }
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void generate_coordinate_system(int voxelLocationIndex, const Vector3f *surfaceNormals, Vector3f *xAxes, Vector3f *yAxes)
{
  Vector3f n = surfaceNormals[voxelLocationIndex];
  Vector3f xAxis = generate_arbitrary_coplanar_unit_vector(n);
  xAxes[voxelLocationIndex] = xAxis;
  yAxes[voxelLocationIndex] = cross(xAxis, n);
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void generate_rgb_patch(int voxelLocationIndex, const Vector3s *voxelLocations, const Vector3f *xAxes, const Vector3f *yAxes,
                               const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData, size_t patchSize, float patchSpacing,
                               size_t featureCount, float *features)
{
  // Get the location of the voxel at the centre of the patch.
  Vector3f centre = voxelLocations[voxelLocationIndex].toFloat();

  // Generate an RGB patch around the voxel on a patchSize * patchSize grid aligned with the voxel's x and y axes.
  int halfPatchSize = static_cast<int>(patchSize - 1) / 2;
  bool isFound;
  Vector3f xAxis = xAxes[voxelLocationIndex] * patchSpacing;
  Vector3f yAxis = yAxes[voxelLocationIndex] * patchSpacing;

  // For each pixel in the patch:
  size_t offset = voxelLocationIndex * featureCount;
  for(int y = -halfPatchSize; y <= halfPatchSize; ++y)
  {
    Vector3f yLoc = centre + static_cast<float>(y) * yAxis;
    for(int x = -halfPatchSize; x <= halfPatchSize; ++x)
    {
      // Compute the location of the pixel in world space.
      Vector3i loc = (yLoc + static_cast<float>(x) * xAxis).toIntRound();

      // If there is a voxel at that location, get its colour; otherwise, default to magenta.
      Vector3u clr(255, 0, 255);
      SpaintVoxel voxel = readVoxel(voxelData, indexData, loc, isFound);
      if(isFound) clr = voxel.clr;

      // Write the colour values into the relevant places in the features array.
      features[offset++] = clr.r;
      features[offset++] = clr.g;
      features[offset++] = clr.b;
    }
  }
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void write_surface_normal(int voxelLocationIndex, const Vector3s *voxelLocations, const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                 Vector3f *surfaceNormals)
{
  surfaceNormals[voxelLocationIndex] = computeSingleNormalFromSDF(voxelData, indexData, voxelLocations[voxelLocationIndex].toFloat());
}

}

#endif
