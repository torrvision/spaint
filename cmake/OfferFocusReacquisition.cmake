#################################
# OfferFocusReacquisition.cmake #
#################################

OPTION(USE_FOCUS_REACQUISITION "Use focus reacquisition?" ON)

IF(USE_FOCUS_REACQUISITION)
  ADD_DEFINITIONS(-DUSE_FOCUS_REACQUISITION)
ENDIF()
