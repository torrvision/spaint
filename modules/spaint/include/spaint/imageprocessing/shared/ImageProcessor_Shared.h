/**
 * spaint: ImageProcessor_Shared.h
 */

#ifndef H_SPAINT_IMAGEPROCESSOR_SHARED
#define H_SPAINT_IMAGEPROCESSOR_SHARED

namespace spaint {

//#################### SHARED HELPER FUNCTIONS ####################

/**
 * \brief Calculates the absolute difference between the corresponding pixels of two depth images, 
 *        provided both pixel values are greater than or equal to zero. If either input pixel is 
 *        less than zero then the corresponding output pixel will be set to -1.
 *
 * \param rowMajorIndex   The row-major index of the two pixels on which the operation is being performed.
 * \param firstInputData  The data for the first input image (in row-major format).
 * \param secondInputData The data for the second input image (in row-major format).
 * \param width           The width of each image.
 * \param height          The height of each image.
 * \param outputData      The data for the output image (in column-major format).
 */
_CPU_AND_GPU_CODE_
inline void calculate_pixel_depth_difference(int rowMajorIndex, const float *firstInputData, const float *secondInputData, int width, int height, float *outputData)
{
  int row = rowMajorIndex / width, col = rowMajorIndex % width;
  int columnMajorIndex = col * height + row;
  float firstPixel = firstInputData[rowMajorIndex], secondPixel = secondInputData[rowMajorIndex];
  outputData[columnMajorIndex] = firstPixel >= 0 && secondPixel >= 0 ? fabs(firstPixel - secondPixel) : -1.0f;
}

/**
 * \brief Copies a pixel value from an ArrayFire image to an InfiniTAM image.
 *
 * \param columnMajorIndex  The column-major index of the pixel to be copied.
 * \param inputData         The data for the input image (in column-major format).
 * \param width             The width of each image.
 * \param height            The height of each image.
 * \param outputData        The data for the output image (in row-major format).
 */
_CPU_AND_GPU_CODE_
inline void copy_af_pixel_to_itm(int columnMajorIndex, const unsigned char *inputData, int width, int height, unsigned char *outputData)
{
  int row = columnMajorIndex % height, col = columnMajorIndex / height;
  int rowMajorIndex = row * width + col;
  outputData[rowMajorIndex] = inputData[columnMajorIndex];
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline void copy_itm_pixel_to_af(int rowMajorIndex, const Vector4u *inputData, int width, int height,
                                 unsigned char *outputRedData, unsigned char *outputGreenData,
                                 unsigned char *outputBlueData, unsigned char *outputAlphaData)
{
  int row = rowMajorIndex / width, col = rowMajorIndex % width;
  int columnMajorIndex = col * height + row;
  Vector4u inputPixel = inputData[rowMajorIndex];
  outputRedData[columnMajorIndex] = inputPixel.r;
  outputGreenData[columnMajorIndex] = inputPixel.g;
  outputBlueData[columnMajorIndex] = inputPixel.b;
  outputAlphaData[columnMajorIndex] = inputPixel.a;
}

/**
 * \brief Tests the value of a pixel in an input image against a threshold using the specified comparison operator,
 *        and either writes a specified value to the corresponding pixel in the output image (if the test is passed),
 *        or copies the value of the input pixel across (otherwise).
 *
 * \param pixelIndex  The index of the pixel being tested.
 * \param inputData   The data for the input image.
 * \param op          The comparison operator.
 * \param threshold   The value against which to compare the pixel value.
 * \param value       The value to which to set the pixel in the output image when the input pixel passes the test.
 * \param outputData  The data for the output image.
 */
_CPU_AND_GPU_CODE_
inline void set_pixel_on_threshold(int pixelIndex, const float *inputData, ImageProcessor::ComparisonOperator op, float threshold, float value, float *outputData)
{
  float input = inputData[pixelIndex];
  float *output = outputData + pixelIndex;

  switch(op)
  {
    case ImageProcessor::CO_GREATER:
    {
      *output = input > threshold ? value : input;
      break;
    }
    case ImageProcessor::CO_LESS:
    {
      *output = input < threshold ? value : input;
      break;
    }
    default:
    {
      // This should never happen.
      break;
    }
  }
}

}

#endif
