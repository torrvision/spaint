################
# UseOVR.cmake #
################

OPTION(WITH_OVR "Build with Oculus support?" OFF)

IF(WITH_OVR)
  IF(MSVC_IDE)
    FIND_PATH(OVR_INCLUDE_DIR OVR.h HINTS $ENV{HOMEPATH}/Downloads/ovr_sdk_win_0.4.2/OculusSDK/LibOVR/Include)
    FIND_LIBRARY(OVR_LIBRARY libovr64 HINTS $ENV{HOMEPATH}/Downloads/ovr_sdk_win_0.4.2/OculusSDK/LibOVR/Lib/x64/VS2013)
  ELSEIF(APPLE)
    FIND_PATH(OVR_INCLUDE_DIR OVR.h HINTS ~/Downloads/OculusSDK/LibOVR/Include)
    FIND_LIBRARY(OVR_LIBRARY ovr HINTS ~/Downloads/OculusSDK/LibOVR/Lib/Mac/Release)
  ELSE()
    FIND_PATH(OVR_INCLUDE_DIR OVR.h)
    FIND_LIBRARY(OVR_LIBRARY ovr)
  ENDIF()

  IF(APPLE)
    FIND_LIBRARY(COREFOUNDATION CoreFoundation)
    FIND_LIBRARY(COREGRAPHICS CoreGraphics)
    FIND_LIBRARY(IOKIT IOKit)
  ENDIF()

  INCLUDE_DIRECTORIES(${OVR_INCLUDE_DIR})
ENDIF()
