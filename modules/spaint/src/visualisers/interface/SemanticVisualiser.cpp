/**
 * spaint: SemanticVisualiser.cpp
 */

#include "visualisers/interface/SemanticVisualiser.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

SemanticVisualiser::SemanticVisualiser(const std::vector<Vector3u>& labelColours)
: m_labelColoursMB(static_cast<int>(labelColours.size()), true, true)
{
  Vector3u *labelColoursData = m_labelColoursMB.GetData(MEMORYDEVICE_CPU);
  for(size_t i = 0, size = labelColours.size(); i < size; ++i)
  {
    labelColoursData[i] = labelColours[i];
  }
  m_labelColoursMB.UpdateDeviceFromHost();
}

//#################### DESTRUCTOR ####################

SemanticVisualiser::~SemanticVisualiser() {}

}
