/**
 * spaint: PNGUtil.h
 */

#ifndef H_SPAINT_PNGUTIL
#define H_SPAINT_PNGUTIL

#include <vector>

#include <boost/shared_ptr.hpp>

#include <ITMLib/Utils/ITMImageTypes.h>

namespace spaint {

/**
 * \brief This class contains utility functions for loading and saving PNG images.
 */
class PNGUtil
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<ITMUChar4Image> ITMUChar4Image_Ptr;
  typedef boost::shared_ptr<const ITMUChar4Image> ITMUChar4Image_CPtr;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Loads an RGBA image from a PNG file.
   *
   * \param path  The path to the file from which to load the image.
   * \return      The loaded image.
   */
  static ITMUChar4Image_Ptr load_rgba_image(const std::string& path);

  /**
   * \brief Saves an RGBA image to a PNG file.
   *
   * \param image The image to save.
   * \param path  The path to the file to which to save it.
   */
  static void save_image(const ITMUChar4Image_CPtr& image, const std::string& path);

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Decodes a buffer in RGBA PNG format into an image.
   *
   * \param buffer  The buffer to decode.
   * \param path    The name of the file from which the buffer was originally loaded (if known).
   * \return        The decoded image.
   */
  static ITMUChar4Image_Ptr decode_rgba_png(const std::vector<unsigned char>& buffer, const std::string& path);

  /**
   * \brief Encodes an RGBA image in PNG format and writes it into a buffer.
   *
   * \param image   The image to encode.
   * \param buffer  The buffer into which to write the encoded image.
   */
  static void encode_png(const ITMUChar4Image_CPtr& image, std::vector<unsigned char>& buffer);
};

}

#endif
