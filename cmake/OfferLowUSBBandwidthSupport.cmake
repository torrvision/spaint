#####################################
# OfferLowUSBBandwidthSupport.cmake #
#####################################

OPTION(USE_LOW_USB_BANDWIDTH_MODE "Use low USB bandwidth mode?" OFF)

IF(USE_LOW_USB_BANDWIDTH_MODE)
  ADD_DEFINITIONS(-DUSE_LOW_USB_BANDWIDTH_MODE)
ENDIF()
