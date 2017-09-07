/**
 * grove: CPURNG.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "numbers/CPURNG.h"

namespace grove {

//#################### CONSTRUCTORS ####################

CPURNG::CPURNG(unsigned int seed)
: m_gen(seed)
{}

}
