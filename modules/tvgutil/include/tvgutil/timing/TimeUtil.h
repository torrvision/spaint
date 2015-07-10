/**
 * tvgutil: TimeUtil.h
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
public:
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
};

}

#endif
