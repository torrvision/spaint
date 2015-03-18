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
 * \brief An instance of a class deriving from this one can be used to apply image processing algorithms to images.
 *
 */
class ImageProcessing
{
  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the image processor.
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

   /**
   * \brief Calculates the pixel-wise absolute difference between two images.
   *
   * \param outputImage      The image holding the result of the calculation.
   * \param firstInputImage  The first image.
   * \param secondInputImage The second image.
   */
 virtual void absolute_difference_calculator(af::array *outputImage, ITMFloatImage *firstInputImage, ITMFloatImage *secondInputImage) const = 0;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /*
   * \brief Checks whether two images have the same size.
   *
   * \param imgA  The firest image.
   * \param imgB  The second image.
   */
  static void check_image_size_equal(ITMFloatImage *imgA, ITMFloatImage *imgB)
  {
    check_equal(imgA->noDims.x, imgB->noDims.x);
    check_equal(imgA->noDims.y, imgB->noDims.y);
  }

 /*
   * \brief Checks whether two images have the same size.
   *
   * \param imgA  The firest image.
   * \param imgB  The second image.
   */
 static void check_image_size_equal(af::array *imgA, ITMFloatImage *imgB)
  {
    check_equal(imgA->dims(0), imgB->noDims.y);
    check_equal(imgA->dims(1), imgB->noDims.x);
  }

  /**
   * \brief Calculates a column-major index from a row-major index for a 2D matrix.
   *
   * \param rowMajorIndex  The row-major index.
   * \param width          The width of the matrix.
   * \param heidht         The height of the matrix.
   * \return               The column-major index.
   */
  static int column_major_index_from_row_major_index(int rowMajorIndex, int width, int height)
  {
    int row = rowMajorIndex / width;
    int col = rowMajorIndex % width;
    return col * height + row;
  }

 //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
 /**
  * \brief Checks whether two integers are equal.
  * Note: Throws an error if the two integers are not equal.
  *
  * \param a  The first integer.
  * \param b  The second integer.
  */
  static void check_equal(int a, int b)
  {
    if(a == b) return;
    else throw std::runtime_error("The image dimensions are not equal.\n");
  }
};

}

#endif
