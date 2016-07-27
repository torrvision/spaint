/**
 * spaint: ColourAppearanceModel.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "segmentation/ColourAppearanceModel.h"

#include "util/ColourConversion_Shared.h"
using namespace rafl;

namespace spaint {

//#################### CONSTRUCTORS ####################

ColourAppearanceModel::ColourAppearanceModel(int binsCb, int binsCr)
: m_binsCb(binsCb), m_binsCr(binsCr)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

float ColourAppearanceModel::compute_posterior_probability(const Vector3u& rgbColour) const
{
  // If we haven't yet seen enough training data to successfully build our appearance model, early out.
  if(!m_pmfColourGivenObject || !m_pmfColourGivenNotObject) return 0.5f;

  /*
  P(object | colour) =                   P(colour | object) * P(object)
                       -----------------------------------------------------------------
                       P(colour | object) * P(object) + P(colour | !object) * P(!object)

  For simplicity, assume that P(object) = P(!object) = 0.5. Then:

  P(object | colour) =            P(colour | object)
                       ----------------------------------------
                       P(colour | object) + P(colour | !object)
  */
  int bin = compute_bin(rgbColour);
  float colourGivenObject = m_pmfColourGivenObject->get_mass(bin);
  float colourGivenNotObject = m_pmfColourGivenNotObject->get_mass(bin);
  float denom = colourGivenObject + colourGivenNotObject;
  return denom > 0.0f ? colourGivenObject / denom : 0.5f;
}

void ColourAppearanceModel::train(const ITMUChar4Image_CPtr& image, const ITMUCharImage_CPtr& objectMask)
{
  // Update the likelihood histograms based on the colour image and object mask.
  const Vector4u *imagePtr = image->GetData(MEMORYDEVICE_CPU);
  const uchar *objectMaskPtr = objectMask->GetData(MEMORYDEVICE_CPU);
  for(int i = 0, size = static_cast<int>(image->dataSize); i < size; ++i)
  {
    int bin = compute_bin(imagePtr[i].toVector3());
    (objectMaskPtr[i] ? m_histColourGivenObject : m_histColourGivenNotObject).add(bin);
  }

  // Update the likelihood PMFs from the histograms.
  if(m_histColourGivenObject.get_count() > 0) m_pmfColourGivenObject.reset(new ProbabilityMassFunction<int>(m_histColourGivenObject));
  if(m_histColourGivenNotObject.get_count() > 0) m_pmfColourGivenNotObject.reset(new ProbabilityMassFunction<int>(m_histColourGivenNotObject));
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

int ColourAppearanceModel::compute_bin(const Vector3u& rgbColour) const
{
  Vector3f yccColour = convert_rgb_to_ycbcr(rgbColour);
  float cbFrac = yccColour.y / 255.0f;
  float crFrac = yccColour.z / 255.0f;
  int x = (int)CLAMP(ROUND(cbFrac * (m_binsCb - 1)), 0, m_binsCb - 1);
  int y = (int)CLAMP(ROUND(crFrac * (m_binsCr - 1)), 0, m_binsCr - 1);
  return y * m_binsCb + x;
}

}
