/**
 * itmx: SemanticMaskingImageSourceEngine.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2019. All rights reserved.
 */

#ifndef H_ITMX_SEMANTICMASKINGIMAGESOURCEENGINE
#define H_ITMX_SEMANTICMASKINGIMAGESOURCEENGINE

#include <set>

#include <orx/base/ORImagePtrTypes.h>

#include "../base/ITMObjectPtrTypes.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to yield RGB-D images in which pixels with particular semantic labels have been masked out.
 */
class SemanticMaskingImageSourceEngine : public InputSource::ImageSourceEngine
{
  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief The values of this enumeration denote the different types of masking that can be used.
   */
  enum MaskingType
  {
    /** Mask both the RGB and depth images. */
    MASK_BOTH,

    /** Only mask the depth images. */
    MASK_DEPTH_ONLY,

    /** Only mask the RGB images. */
    MASK_RGB_ONLY
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The colours that should be masked out. */
  std::vector<Vector3u> m_coloursToMask;

  /** A map from semantic labels to colours in the semantic images. */
  std::map<std::string,Vector3u> m_labelToColourMap;

  /** The type of masking to use. */
  MaskingType m_maskingType;

  /** The image source from which to obtain the normal images. */
  ImageSourceEngine_Ptr m_normalSource;

  /** A temporary image in which to store the most recent semantic image. */
  ORUChar4Image_Ptr m_semanticImage;

  /** The image source from which to obtain the semantic images. */
  ImageSourceEngine_Ptr m_semanticSource;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a semantic masking image source engine.
   *
   * \param normalSource    The image source from which to obtain the normal images.
   * \param semanticSource  The image source from which to obtain the semantic images.
   * \param maskingType     The type of masking to use.
   * \param labelsToMask    The semantic labels that should be masked out.
   * \param labelSet        The set of semantic labels to use (currently only "cityscapes" is supported).
   */
  SemanticMaskingImageSourceEngine(ImageSourceEngine *normalSource, ImageSourceEngine *semanticSource, MaskingType maskingType,
                                   const std::set<std::string>& labelsToMask, const std::string& labelSet = "cityscapes");

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual ITMLib::ITMRGBDCalib getCalib() const;

  /** Override */
  virtual Vector2i getDepthImageSize() const;

  /** Override */
  virtual void getImages(ORUChar4Image *rgb, ORShortImage *rawDepth);

  /** Override */
  virtual Vector2i getRGBImageSize() const;

  /** Override */
  virtual bool hasMoreImages() const;
};

}

#endif
