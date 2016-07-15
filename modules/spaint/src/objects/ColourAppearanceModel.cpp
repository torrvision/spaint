/**
 * spaint: ColourAppearanceModel.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "objects/ColourAppearanceModel.h"
using namespace rafl;

namespace spaint {

//#################### CONSTRUCTORS ####################

ColourAppearanceModel::ColourAppearanceModel(int binsCb, int binsCr)
: m_binsCb(binsCb), m_binsCr(binsCr)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

double ColourAppearanceModel::compute_posterior_probability(const Vector3u& rgbColour) const
{
  /*
  P(object | colour) =                   P(colour | object) * P(object)
                       -----------------------------------------------------------------
                       P(colour | object) * P(object) + P(colour | ¬object) * P(¬object)

  For simplicity, assume that P(object) = P(¬object) = 0.5. Then:

  P(object | colour) =          2 * P(colour | object)
                       ----------------------------------------
                       P(colour | object) + P(colour | ¬object)
  */
  int bin = compute_bin(rgbColour);
  float colourGivenObject = m_pmfColourGivenObject->get_mass(bin);
  float colourGivenNotObject = m_pmfColourGivenNotObject->get_mass(bin);
  return 2.0f * colourGivenObject / (colourGivenObject + colourGivenNotObject);
}

void ColourAppearanceModel::update(const ITMUChar4Image_CPtr& image, const ITMUCharImage_CPtr& objectMask)
{
  // Update the likelihood histograms based on the colour image and object mask.
  const Vector4u *imagePtr = image->GetData(MEMORYDEVICE_CPU);
  const uchar *objectMaskPtr = objectMask->GetData(MEMORYDEVICE_CPU);
  for(int i = 0, size = static_cast<int>(image->dataSize); i < size; ++i)
  {
    int bin = compute_bin(imagePtr[i].toVector3());

    if(objectMaskPtr[i]) m_histColourGivenObject.add(bin);
    else m_histColourGivenNotObject.add(bin);
  }

  // Update the likelihood PMFs from the histograms.
  m_pmfColourGivenObject.reset(new ProbabilityMassFunction<int>(m_histColourGivenObject));
  m_pmfColourGivenNotObject.reset(new ProbabilityMassFunction<int>(m_histColourGivenNotObject));
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

int ColourAppearanceModel::compute_bin(const Vector3u& rgbColour) const
{
  // TODO
  throw 23;
}

}
