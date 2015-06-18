###############################
# OfferLowMemorySupport.cmake #
###############################

OPTION(USE_LOW_MEMORY_MODE "Use low-memory mode?" OFF)

IF(USE_LOW_MEMORY_MODE)
  ADD_DEFINITIONS(-DUSE_LOW_MEMORY_MODE)
ENDIF()
