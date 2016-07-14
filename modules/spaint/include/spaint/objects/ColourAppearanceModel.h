/**
 * spaint: ColourAppearanceModel.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_COLOURAPPEARANCEMODEL
#define H_SPAINT_COLOURAPPEARANCEMODEL

#include <boost/shared_ptr.hpp>

#include <ITMLib/Utils/ITMImageTypes.h>

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

  //#################### PRIVATE VARIABLES ####################
private:
  // TODO

  //#################### CONSTRUCTORS ####################
public:
  // TODO

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
  void update(const ITMUChar4Image_CPtr& image, const ITMUCharImage_CPtr& objectMask);
};

}

#endif
