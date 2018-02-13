/**
 * itmx: FallibleTracker.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_ITMX_FALLIBLETRACKER
#define H_ITMX_FALLIBLETRACKER

#include <boost/shared_ptr.hpp>

#include <ITMLib/Trackers/Interface/ITMTracker.h>

namespace itmx {

/**
 * \brief An instance of a class deriving from this one can be used to track the camera pose in a way that allows it to detect tracking failures.
 */
class FallibleTracker : public ITMLib::ITMTracker
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** A flag recording whether or not we have temporarily lost tracking. */
  bool m_lostTracking;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a fallible tracker.
   */
  FallibleTracker()
  : m_lostTracking(false)
  {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets whether or not we have temporarily lost tracking.
   *
   * \return  true, if we have temporarily lost tracking, or false otherwise.
   */
  bool lost_tracking() const
  {
    return m_lostTracking;
  }
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<FallibleTracker> FallibleTracker_Ptr;

}

#endif
