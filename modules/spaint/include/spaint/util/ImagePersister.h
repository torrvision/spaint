/**
 * spaint: ImagePersister.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_IMAGEPERSISTER
#define H_SPAINT_IMAGEPERSISTER

#include <vector>

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ITMLib/Utils/ITMImageTypes.h>

namespace spaint {

/**
 * \brief This class contains utility functions for loading and saving images.
 */
class ImagePersister
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMShortImage> ITMShortImage_CPtr;
  typedef boost::shared_ptr<ITMUChar4Image> ITMUChar4Image_Ptr;
  typedef boost::shared_ptr<const ITMUChar4Image> ITMUChar4Image_CPtr;

  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief The values of this enumeration represent the supported image file types.
   */
  enum ImageFileType
  {
    IFT_PGM,
    IFT_PNG,
    IFT_PPM,
    IFT_UNKNOWN
  };

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Attempts to load an RGBA image from a file.
   *
   * \param path                The path to the file from which to load the image.
   * \param fileType            The image file type.
   * \return                    The loaded image.
   * \throws std::runtime_error If the image could not be loaded.
   */
  static ITMUChar4Image_Ptr load_rgba_image(const std::string& path, ImageFileType fileType = IFT_UNKNOWN);

  /**
   * \brief Attempts to save a short image to a file.
   *
   * \param image               The image to save.
   * \param path                The path to the file to which to save it.
   * \param fileType            The image file type.
   * \throws std::runtime_error If the image could not be saved.
   */
  static void save_image(const ITMShortImage_CPtr& image, const std::string& path, ImageFileType fileType = IFT_UNKNOWN);

  /**
   * \brief Attempts to save an RGBA image to a file.
   *
   * \param image               The image to save.
   * \param path                The path to the file to which to save it.
   * \param fileType            The image file type.
   * \throws std::runtime_error If the image could not be saved.
   */
  static void save_image(const ITMUChar4Image_CPtr& image, const std::string& path, ImageFileType fileType = IFT_UNKNOWN);

  /**
   * \brief Attempts to save an image to a file on a separate thread.
   *
   * \param image               The image to save.
   * \param path                The path to the file to which to save it.
   * \param fileType            The image file type.
   * \throws std::runtime_error If the image could not be saved.
   */
  template <typename T>
  static void save_image_on_thread(const boost::shared_ptr<const ORUtils::Image<T> >& image, const std::string& path, ImageFileType fileType = IFT_UNKNOWN)
  {
    void (*p)(const boost::shared_ptr<const ORUtils::Image<T> >&, const std::string&, ImageFileType) = &save_image;
    boost::thread t(p, image, path, fileType);
    t.detach();
  }

  /**
   * \brief Attempts to save an image to a file on a separate thread.
   *
   * \param image               The image to save.
   * \param path                The path to the file to which to save it.
   * \param fileType            The image file type.
   * \throws std::runtime_error If the image could not be saved.
   */
  template <typename T>
  static void save_image_on_thread(const boost::shared_ptr<const ORUtils::Image<T> >& image, const boost::filesystem::path& path, ImageFileType fileType = IFT_UNKNOWN)
  {
    save_image_on_thread(image, path.string(), fileType);
  }

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
   * \brief Attempts to deduce an image file's type based on its file extension.
   *
   * \param path                The path to the image file.
   * \return                    The image file's type, if it can be deduced from the file extension, or IFT_UNKNOWN otherwise.
   */
  static ImageFileType deduce_image_file_type(const std::string& path);

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
