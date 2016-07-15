/**
 * spaint: ColourAppearanceModel.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_COLOURAPPEARANCEMODEL
#define H_SPAINT_COLOURAPPEARANCEMODEL

#include <boost/shared_ptr.hpp>

#include <ITMLib/Utils/ITMImageTypes.h>

// FIXME: There's a reasonable argument that things like Histogram and ProbabilityMassFunction should be moved somewhere more central.
#include <rafl/base/Histogram.h>
#include <rafl/base/ProbabilityMassFunction.h>

namespace spaint {

/**
 * \brief An instance of this class can be used to represent a pixel-wise colour appearance model for an object.
 */
class ColourAppearanceModel
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const ITMUCharImage> ITMUCharImage_CPtr;
  typedef boost::shared_ptr<const ITMUChar4Image> ITMUChar4Image_CPtr;
  typedef boost::shared_ptr<rafl::ProbabilityMassFunction<int> > PMF_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** TODO */
  int m_binsCb;

  /** TODO */
  int m_binsCr;

  // P(Colour | object)
  rafl::Histogram<int> m_histColourGivenObject;

  // P(Colour | ¬object)
  rafl::Histogram<int> m_histColourGivenNotObject;

  // P(Colour | object)
  PMF_Ptr m_pmfColourGivenObject;

  // P(Colour | ¬object)
  PMF_Ptr m_pmfColourGivenNotObject;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   *
   * \param binsCb  TODO
   * \param binsCr  TODO
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
  double compute_posterior_probability(const Vector3u& rgbColour) const;

  /**
   * \brief TODO
   */
  void train(const ITMUChar4Image_CPtr& image, const ITMUCharImage_CPtr& objectMask);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  int compute_bin(const Vector3u& rgbColour) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<ColourAppearanceModel> ColourAppearanceModel_Ptr;

}

#endif
