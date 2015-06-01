/**
 * spaint: ImageProcessor.h
 */

#ifndef H_SPAINT_IMAGEPROCESSOR
#define H_SPAINT_IMAGEPROCESSOR

#include <arrayfire.h>

#include <boost/shared_ptr.hpp>

#include <ITMLib/Utils/ITMLibDefines.h>

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to apply image processing algorithms to images.
 */
class ImageProcessor
{
  //#################### TYPEDEFS ####################
protected:
  typedef boost::shared_ptr<af::array> AFImage_Ptr;
  typedef boost::shared_ptr<const af::array> AFImage_CPtr;
  typedef boost::shared_ptr<ITMFloatImage> ITMFloatImage_Ptr;
  typedef boost::shared_ptr<const ITMFloatImage> ITMFloatImage_CPtr;

  //#################### ENUMERATIONS ####################
public: 
  /**
   * \brief An enumeration containing the possible comparison operators that can be used when testing pixel values.
   */
  enum ComparisonOperator
  {
    CO_GREATER,
    CO_LESS
  };

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the image processor.
   */
  virtual ~ImageProcessor();

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
   /**
   * \brief Calculates the pixel-wise absolute difference between two depth images.
   *
   * Note that either input image may contain pixels with invalid values (less than zero).
   * For each pixel-wise operation, if either input pixel is invalid then the corresponding
   * output pixel will be set to -1.
   *
   * \param firstInputImage  The first input image.
   * \param secondInputImage The second input image.
   * \param outputImage      The image in which to store the result of the calculation.
   */
  virtual void calculate_depth_difference(const ITMFloatImage_CPtr& firstInputImage, const ITMFloatImage_CPtr& secondInputImage, const AFImage_Ptr& outputImage) const = 0;

  /**
   * \brief Tests the pixels in the input image against a threshold using the specified comparison operator,
   *        and makes a copy of the image in which the corresponding pixels are either set to the specified
   *        value (if they pass the test), or to their value in the input image (otherwise).
   *
   * \param inputImage  The input image.
   * \param op          The comparison operator.
   * \param threshold   The value against which to compare the pixel values.
   * \param value       The value to which to set a pixel in the output image when its corresponding input pixel passes the test.
   * \param outputImage The output image.
   */
  virtual void set_on_threshold(const ITMFloatImage_CPtr& inputImage, ComparisonOperator op, float threshold, float value, const ITMFloatImage_Ptr& outputImage) const = 0;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /*
   * \brief Checks whether two images have the same size.
   *
   * \param imgA  The first image.
   * \param imgB  The second image.
   */
  template <typename T, typename U>
  static void check_image_size_equal(const T& imgA, const U& imgB)
  {
    Vector2i sizeA = image_size(imgA);
    Vector2i sizeB = image_size(imgB);
    if(sizeA != sizeB)
    {
      throw std::runtime_error("The image dimensions are not equal");
    }
  }

  /**
   * \brief Calculates a column-major index from a row-major index for a 2D matrix.
   *
   * For example, consider a 3x3 (width x height) matrix A:
   *
   * - The element whose row = 2 and column = 1 will have a row-major index of (row * width + col) = 7.
   * - The same element will have a column-major index of (col * height + row) = 5.
   *
   * To convert from a row-major to a column major index, we first find the row and column indices
   * from the row-major index, and then calculate the column-major index as above.
   *
   * \param rowMajorIndex  The row-major index.
   * \param width          The width of the matrix.
   * \param height         The height of the matrix.
   * \return               The column-major index.
   */
  static int column_major_index_from_row_major_index(int rowMajorIndex, int width, int height);

 //#################### PROTECTED STATIC MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Gets the size of an ArrayFire image.
   *
   * \param img The ArrayFire image.
   * \return    The size of the ArrayFire image.
   */
  static Vector2i image_size(const AFImage_CPtr& img);

  /**
   * \brief Gets the size of an InfiniTAM image.
   *
   * \param img  The InfiniTAM image.
   * \return     The size of the InfiniTAM image.
   */
  static Vector2i image_size(const ITMFloatImage_CPtr& img);
};

}

#endif
