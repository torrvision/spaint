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
SET(core_multiplatform_cpu_sources
src/core/multiplatform/cpu/SemanticRaycastImpl_CPU.cpp
)

SET(core_multiplatform_cpu_headers
include/spaint/core/multiplatform/cpu/SemanticRaycastImpl_CPU.h
)

##
SET(core_multiplatform_cuda_sources
src/core/multiplatform/cuda/SemanticRaycastImpl_CUDA.cu
)

SET(core_multiplatform_cuda_headers
include/spaint/core/multiplatform/cuda/SemanticRaycastImpl_CUDA.h
)

##
SET(core_multiplatform_interface_headers
include/spaint/core/multiplatform/interface/SemanticRaycastImpl.h
)

##
SET(core_voxels_headers
include/spaint/core/voxels/SpaintVoxel.h
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
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${core_sources}
${core_multiplatform_cpu_sources}
${input_sources}
${util_sources}
)

SET(headers
${core_headers}
${core_multiplatform_cpu_headers}
${core_multiplatform_interface_headers}
${core_voxels_headers}
${input_headers}
${ogl_headers}
${util_headers}
)

IF(WITH_CUDA)
  SET(sources ${sources} ${core_multiplatform_cuda_sources})
  SET(headers ${headers} ${core_multiplatform_cuda_headers})
ENDIF()

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(core FILES ${core_sources} ${core_headers})
SOURCE_GROUP(core\\multiplatform\\cpu FILES ${core_multiplatform_cpu_sources} ${core_multiplatform_cpu_headers})
SOURCE_GROUP(core\\multiplatform\\cuda FILES ${core_multiplatform_cuda_sources} ${core_multiplatform_cuda_headers})
SOURCE_GROUP(core\\multiplatform\\interface FILES ${core_multiplatform_interface_headers})
SOURCE_GROUP(core\\voxels FILES ${core_voxels_headers})
SOURCE_GROUP(input FILES ${input_sources} ${input_headers})
SOURCE_GROUP(ogl FILES ${ogl_headers})
SOURCE_GROUP(util FILES ${util_sources} ${util_headers})

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
