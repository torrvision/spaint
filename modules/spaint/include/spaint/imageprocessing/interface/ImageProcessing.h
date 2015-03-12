/**
 * spaint: ImageProcessing.h
 */

#ifndef H_SPAINT_IMAGEPROCESSING
#define H_SPAINT_IMAGEPROCESSING

#include <stdexcept>

#include <arrayfire.h>
#include <ITMLib/Utils/ITMLibDefines.h>

namespace spaint {

/**
 * \brief TODO
 *
 */
class ImageProcessing
{
  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the depth calculator.
   */
  virtual ~ImageProcessing() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Calculates the pixel-wise absolute difference between two images.
   * 
   * \param outputImage      The image holding the result of the calculation.
   * \param firstInputImage  The first image. 
   * \param secondInputImage The second image.
   */
  virtual void absolute_difference_calculator(ITMFloatImage *outputImage, ITMFloatImage *firstInputImage, ITMFloatImage *secondInputImage) const = 0;

  virtual void absolute_difference_calculator(af::array *outputImage, ITMFloatImage *firstInputImage, ITMFloatImage *secondInputImage) const = 0;

  /**
   * \brief Calculates a binary image from an input image by applying a threshold on its pixel values.
   *
   * \param outputImage      The image holding the result of the calculation.
   * \param inputImage       The image on which to carry out the calculation.
   * \param threshold        The threshold used to calculate a binary iamge.
   * \param maxiBinaryValue  The value to set a pixel if it is greater than the specified threshold.
   */
  //virtual void binary_threshold_calculator(ITMFloatImage *outputImage, ITMFloatImage *inputImage, float threshold, float maxBinaryValue) const = 0;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS #################### 
  /*
   * \brief Checks whether two images have the same size.
   * Note: Throws an error if the two images do not have the same size.
   *
   * \imgA  The firest image.
   * \imgB  The second image.
   */
  static void check_image_size_equal(ITMFloatImage *imgA, ITMFloatImage *imgB)
  {
    check_equal(imgA->noDims.x, imgB->noDims.x);
    check_equal(imgA->noDims.y, imgB->noDims.y);
  }

  static void check_image_size_equal(af::array *imgA, ITMFloatImage *imgB)
  {
    check_equal(imgA->dims(0), imgB->noDims.y);
    check_equal(imgA->dims(1), imgB->noDims.x);
  }

private:
  static void check_equal(int a, int b)
  {
    if(a == b) return;
    else throw std::runtime_error("The image dimensions are not equal.\n");
  }
};

}

#endif
