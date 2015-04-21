/**
 * spaint: ImageProcessor.h
 */

#ifndef H_SPAINT_IMAGEPROCESSOR
#define H_SPAINT_IMAGEPROCESSOR

#include <stdexcept>

#include <arrayfire.h>

#include <ITMLib/Utils/ITMLibDefines.h>

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to apply image processing algorithms to images.
 *
 */
class ImageProcessor
{
public: 
  /**
   * \brief Comparison Operators.
   */
  enum ComparisonOperator
  {
    GREATER,
    LESS
  };

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the image processor.
   */
  virtual ~ImageProcessor() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Calculates the pixel-wise absolute difference between two images.
   *
   * \param outputImage      The image holding the result of the calculation.
   * \param firstInputImage  The first input image.
   * \param secondInputImage The second input image.
   */
  virtual void absolute_difference_calculator(ITMFloatImage *outputImage, const ITMFloatImage *firstInputImage, const ITMFloatImage *secondInputImage) const = 0;

   /**
   * \brief Calculates the pixel-wise absolute difference between two images.
   *
   * \param outputImage      The image holding the result of the calculation.
   * \param firstInputImage  The first input image.
   * \param secondInputImage The second input image.
   */
  virtual void absolute_difference_calculator(af::array *outputImage, const ITMFloatImage *firstInputImage, const ITMFloatImage *secondInputImage) const = 0;

  /**
   * \brief Sets pixels to a specified value if a condition is satisfied.
   *
   * \param output     The output image.
   * \param input      The input image.
   * \param comparator The value to compare the pixel value to.
   * \param value      The value to set the pixel to if the comparison is true.
   */
  virtual void pixel_setter(ITMFloatImage *output, const ITMFloatImage *input, float comparator, ComparisonOperator comparisonOperator, float value) const = 0;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /*
   * \brief Checks whether two images have the same size.
   *
   * \param imgA  The first image.
   * \param imgB  The second image.
   */
  template <typename T, typename U>
  static void check_image_size_equal(const T* imgA, const U* imgB)
  {
    check_equal(image_size(imgA), image_size(imgB));
  }

  /**
   * \brief Calculates a column-major index from a row-major index for a 2D matrix.
   *
   * Consider a 3x3 (width x height) matrix A.
   * The element (2, 1) (row, col) will have a row-major index of RM = (row * width + col) = 7.
   * The same element will have a column-major index of CM = (col * height + row) = 5.
   * In order to convert from row-major to column major indices, 
   * first find the row and column indices from the row major index, 
   * and then calculate CM as above.
   *
   * \param rowMajorIndex  The row-major index.
   * \param width          The width of the matrix.
   * \param height         The height of the matrix.
   * \return               The column-major index.
   */
  static int column_major_index_from_row_major_index(int rowMajorIndex, int width, int height);

 //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
 /**
  * \brief Checks whether two vectors are equal.
  *
  * \throws Throws an error if the vectors are not equal.
  *
  * \param a  The first vector.
  * \param b  The second vector.
  */
  static void check_equal(const std::vector<int>& a, const std::vector<int>& b);

  /**
   * \brief Gets the image size from an ArrayFire image.
   *
   * \param img The Arrayfire image.
   * \return    The size of the ArrayFire image.
   */
  static std::vector<int> image_size(const af::array *img);

  /**
   * \brief Gets the image size from an InfiniTAM image.
   *
   * \param img  The InfiniTAM image.
   * \return     The size of the InfiniTAM image.
   */
  static std::vector<int> image_size(const ITMFloatImage* img);
};

}

#endif

