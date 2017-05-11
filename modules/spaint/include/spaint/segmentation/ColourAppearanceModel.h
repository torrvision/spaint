/**
 * spaint: ColourAppearanceModel.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_COLOURAPPEARANCEMODEL
#define H_SPAINT_COLOURAPPEARANCEMODEL

#include <itmx/base/ITMImagePtrTypes.h>

#include <tvgutil/statistics/ProbabilityMassFunction.h>

namespace spaint {

/**
 * \brief An instance of this class can be used to represent a pixel-wise colour appearance model for an object.
 *
 * We base our model on a chroma-based 2D histogram over colours in the YCbCr colour space.
 */
class ColourAppearanceModel
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<tvgutil::ProbabilityMassFunction<int> > PMF_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The number of Cb bins in the histogram. */
  int m_binsCb;

  /** The number of Cr bins in the histogram. */
  int m_binsCr;

  // A (linearised) 2D histogram representing P(Colour | object).
  tvgutil::Histogram<int> m_histColourGivenObject;

  // A (linearised) 2D histogram representing P(Colour | !object).
  tvgutil::Histogram<int> m_histColourGivenNotObject;

  // A (linearised) 2D probability mass function representing P(Colour | object).
  PMF_Ptr m_pmfColourGivenObject;

  // A (linearised) 2D probability mass function representing P(Colour | !object).
  PMF_Ptr m_pmfColourGivenNotObject;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a histogram-based colour appearance model.
   *
   * \param binsCb  The number of Cb bins in the histogram.
   * \param binsCr  The number of Cr bins in the histogram.
   */
  ColourAppearanceModel(int binsCb, int binsCr);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Computes the posterior probability of a pixel being part of the object given its colour, i.e. P(object | colour).
   *
   * \param rgbColour The RGB colour of the pixel.
   * \return          The posterior probability of the pixel being part of the object given its colour.
   */
  float compute_posterior_probability(const Vector3u& rgbColour) const;

  /**
   * \brief Trains the colour appearance model for the object.
   *
   * \param image       An image containing the object and its background.
   * \param objectMask  A binary mask specifying which pixels in the image are object and which are background.
   */
  void train(const ITMUChar4Image_CPtr& image, const ITMUCharImage_CPtr& objectMask);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Computes the 2D histogram bin index for the specified RGB colour.
   *
   * \param rgbColour An RGB colour.
   * \return          The 2D histogram bin index for the colour.
   */
  int compute_bin(const Vector3u& rgbColour) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<ColourAppearanceModel> ColourAppearanceModel_Ptr;

}

#endif
