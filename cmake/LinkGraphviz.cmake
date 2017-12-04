######################
# LinkGraphviz.cmake #
######################

IF(WITH_GRAPHVIZ)
  TARGET_LINK_LIBRARIES(${targetname} ${GRAPHVIZ_LIBRARIES})
ENDIF()
