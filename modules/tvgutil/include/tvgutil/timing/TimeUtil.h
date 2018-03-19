/**
 * tvgutil: TimeUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_TIMEUTIL
#define H_TVGUTIL_TIMEUTIL

#include <boost/date_time/posix_time/posix_time.hpp>

namespace tvgutil {

/**
 * \brief This class provides utility functions related to time.
 */
struct TimeUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Gets the current time in ISO format.
   *
   * \return  The current time in ISO format.
   */
  static std::string get_iso_timestamp()
  {
    boost::posix_time::ptime currentDateTime(boost::posix_time::second_clock::local_time());
    return boost::posix_time::to_iso_string(currentDateTime);
  }

  /**
   * \brief Gets the time since the epoch (in the specified units).
   *
   * \return  The time since the epoch (in the specified units).
   */
  template <typename Units>
  static std::string get_timestamp()
  {
    return boost::lexical_cast<std::string>(get_time_since_epoch<Units>().count());
  }

  /**
   * \brief Gets the time since the epoch (in the specified units).
   *
   * \return  The time since the epoch (in the specified units).
   */
  template <typename Units>
  static Units get_time_since_epoch()
  {
    return boost::chrono::duration_cast<Units>(boost::chrono::system_clock().now().time_since_epoch());
  }
};

}

#endif
