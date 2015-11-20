#################
# UseLeap.cmake #
#################

OPTION(WITH_LEAP "Build with Leap Motion support?" OFF)

IF(WITH_LEAP)
  IF(MSVC_IDE)
    FIND_PATH(LEAP_ROOT head_sha.txt HINTS "$ENV{HOMEPATH}/Downloads/LeapDeveloperKit_2.2.1+24116_win/LeapSDK")
    FIND_LIBRARY(LEAP_LIBRARY Leap HINTS "${LEAP_ROOT}/lib/x64")
  ELSEIF(APPLE)
    FIND_PATH(LEAP_ROOT head_sha.txt HINTS ~/Downloads/LeapDeveloperKit_2.2.1+24116_mac/LeapSDK)
    FIND_LIBRARY(LEAP_LIBRARY Leap HINTS "${LEAP_ROOT}/lib")
  ELSEIF("${CMAKE_SYSTEM}" MATCHES "Linux")
    FIND_PATH(LEAP_ROOT head_sha.txt HINTS ~/software/LeapDeveloperKit_2.2.1+24116_linux/LeapSDK
                                           ~/Software/LeapDeveloperKit_2.2.1+24116_linux/LeapSDK)
    FIND_LIBRARY(LEAP_LIBRARY Leap HINTS "${LEAP_ROOT}/lib/x64")
  ENDIF()

  FIND_PATH(LEAP_INCLUDE_DIR Leap.h HINTS "${LEAP_ROOT}/include")

  INCLUDE_DIRECTORIES(${LEAP_INCLUDE_DIR})
  ADD_DEFINITIONS(-DWITH_LEAP)
ENDIF()
