/**
 * spaint: TrackerType.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_TRACKERTYPE
#define H_SPAINT_TRACKERTYPE

namespace spaint {

/**
 * \brief The different tracker types we can use.
 */
enum TrackerType
{
  TRACKER_INFINITAM,
  TRACKER_RIFT,
  TRACKER_ROBUSTVICON,
  TRACKER_VICON
};

}

#endif
