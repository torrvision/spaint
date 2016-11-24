/**
 * spaint: ExampleReservoirs.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/interface/ExampleReservoirs.h"
#include "randomforest/interface/ExampleReservoirs.tpp"

namespace spaint
{

template class ExampleReservoirs<Keypoint3DColour, Keypoint3DColour, LeafIndices> ;

}
