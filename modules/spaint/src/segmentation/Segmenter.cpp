/**
 * spaint: Segmenter.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "segmentation/Segmenter.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

Segmenter::Segmenter(const View_CPtr& view)
: m_targetMask(new ITMUCharImage(view->rgb->noDims, true, false)), m_view(view)
{}

//#################### DESTRUCTOR ####################

Segmenter::~Segmenter() {}

}
