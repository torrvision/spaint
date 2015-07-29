/**
 * spaint: MarkingMode.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_MARKINGMODE
#define H_SPAINT_MARKINGMODE

namespace spaint {

/**
 * \brief The values of this enumeration represent the different marking modes that are supported.
 */
enum MarkingMode
{
  /** In forced marking mode, an old label is always overwritten with a new one without a test being performed. */
  FORCED_MARKING,

  /** In normal marking mode, an old label is only overwritten with a new one if the two labels pass a test. */
  NORMAL_MARKING
};

}

#endif
