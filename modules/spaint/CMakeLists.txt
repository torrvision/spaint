#####################################
# CMakeLists.txt for modules/spaint #
#####################################

###########################
# Specify the target name #
###########################

SET(targetname spaint)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenCV.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOVR.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseSDL.cmake)

#############################
# Specify the project files #
#############################

##
SET(core_sources
src/core/SpaintModel.cpp
src/core/SpaintPipeline.cpp
src/core/SpaintRaycaster.cpp
)

SET(core_headers
include/spaint/core/SpaintModel.h
include/spaint/core/SpaintPipeline.h
include/spaint/core/SpaintRaycaster.h
)

##
SET(input_sources
src/input/InputState.cpp
)

SET(input_headers
include/spaint/input/InputState.h
include/spaint/input/MouseButton.h
)

##
SET(ogl_headers
include/spaint/ogl/WrappedGL.h
)

##
SET(util_sources
src/util/CameraPoseConverter.cpp
)

SET(util_headers
include/spaint/util/CameraPoseConverter.h
include/spaint/util/SpaintVoxel.h
)

##
SET(visualisers_multiplatform_cpu_sources
src/visualisers/multiplatform/cpu/SemanticVisualiser_CPU.cpp
)

SET(visualisers_multiplatform_cpu_headers
include/spaint/visualisers/multiplatform/cpu/SemanticVisualiser_CPU.h
)

##
SET(visualisers_multiplatform_cuda_sources
src/visualisers/multiplatform/cuda/SemanticVisualiser_CUDA.cu
)

SET(visualisers_multiplatform_cuda_headers
include/spaint/visualisers/multiplatform/cuda/SemanticVisualiser_CUDA.h
)

##
SET(visualisers_multiplatform_interface_headers
include/spaint/visualisers/multiplatform/interface/SemanticVisualiser.h
)

##
SET(visualisers_multiplatform_shared_headers
include/spaint/visualisers/multiplatform/shared/SemanticVisualiser_Shared.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${core_sources}
${input_sources}
${util_sources}
${visualisers_sources}
${visualisers_multiplatform_cpu_sources}
)

SET(headers
${core_headers}
${input_headers}
${ogl_headers}
${util_headers}
${visualisers_multiplatform_cpu_headers}
${visualisers_multiplatform_interface_headers}
${visualisers_multiplatform_shared_headers}
)

IF(WITH_CUDA)
  SET(sources ${sources} ${visualisers_multiplatform_cuda_sources})
  SET(headers ${headers} ${visualisers_multiplatform_cuda_headers})
ENDIF()

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(core FILES ${core_sources} ${core_headers})
SOURCE_GROUP(input FILES ${input_sources} ${input_headers})
SOURCE_GROUP(ogl FILES ${ogl_headers})
SOURCE_GROUP(util FILES ${util_sources} ${util_headers})
SOURCE_GROUP(visualisers\\multiplatform\\cpu FILES ${visualisers_multiplatform_cpu_sources} ${visualisers_multiplatform_cpu_headers})
SOURCE_GROUP(visualisers\\multiplatform\\cuda FILES ${visualisers_multiplatform_cuda_sources} ${visualisers_multiplatform_cuda_headers})
SOURCE_GROUP(visualisers\\multiplatform\\interface FILES ${visualisers_multiplatform_interface_headers})
SOURCE_GROUP(visualisers\\multiplatform\\shared FILES ${visualisers_multiplatform_shared_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/spaint)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/rigging/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)
#INCLUDE_DIRECTORIES(SYSTEM ${PROJECT_SOURCE_DIR}/helpers)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
