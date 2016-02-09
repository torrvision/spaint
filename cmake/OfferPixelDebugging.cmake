#############################
# OfferPixelDebugging.cmake #
#############################

OPTION(USE_PIXEL_DEBUGGING "Use pixel debugging?" OFF)

IF(USE_PIXEL_DEBUGGING)
  ADD_DEFINITIONS(-DUSE_PIXEL_DEBUGGING)
ENDIF()
