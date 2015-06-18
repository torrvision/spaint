###############################
# OfferLowPowerSupport.cmake #
###############################

OPTION(USE_LOW_POWER_MODE "Use low-power mode?" OFF)

IF(USE_LOW_POWER_MODE)
  ADD_DEFINITIONS(-DUSE_LOW_POWER_MODE)
ENDIF()
