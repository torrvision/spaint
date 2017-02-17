#####################################
# CMakeLists.txt for modules/grove #
#####################################

###########################
# Specify the target name #
###########################

SET(targetname grove)

################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)

#############################
# Specify the project files #
#############################

##
SET(features_sources src/features/FeatureCalculatorFactory.cpp)
SET(features_headers include/grove/features/FeatureCalculatorFactory.h)

##
SET(features_base_headers
include/grove/features/base/Descriptor.h
)

##
SET(features_cpu_sources src/features/cpu/RGBDPatchFeatureCalculator_CPU.cpp)
SET(features_cpu_headers include/grove/features/cpu/RGBDPatchFeatureCalculator_CPU.h)
SET(features_cpu_templates include/grove/features/cpu/RGBDPatchFeatureCalculator_CPU.tpp)

##
SET(features_cuda_sources src/features/cuda/RGBDPatchFeatureCalculator_CUDA.cu)
SET(features_cuda_headers include/grove/features/cuda/RGBDPatchFeatureCalculator_CUDA.h)
SET(features_cuda_templates include/grove/features/cuda/RGBDPatchFeatureCalculator_CUDA.tcu)

##
SET(features_interface_sources src/features/interface/RGBDPatchFeatureCalculator.cpp)
SET(features_interface_headers include/grove/features/interface/RGBDPatchFeatureCalculator.h)
SET(features_interface_templates include/grove/features/interface/RGBDPatchFeatureCalculator.tpp)

##
SET(features_shared_headers include/grove/features/shared/RGBDPatchFeatureCalculator_Shared.h)

##
SET(keypoints_headers
include/grove/keypoints/Keypoint2D.h
include/grove/keypoints/Keypoint3DColour.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${features_sources}
${features_cpu_sources}
${features_interface_sources}
)

SET(headers
${features_headers}
${features_base_headers}
${features_cpu_headers}
${features_interface_headers}
${features_shared_headers}
${keypoints_headers}
)

SET(templates
${features_cpu_templates}
${features_interface_templates}
)

IF(WITH_CUDA)
  SET(sources ${sources} ${features_cuda_sources})
  SET(headers ${headers} ${features_cuda_headers})
  SET(templates ${templates} ${features_cuda_templates})
ENDIF()

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(features FILES ${features_sources} ${features_headers})
SOURCE_GROUP(features\\base FILES ${features_base_headers})
SOURCE_GROUP(features\\cpu FILES ${features_cpu_sources} ${features_cpu_headers} ${features_cpu_templates})
SOURCE_GROUP(features\\cuda FILES ${features_cuda_sources} ${features_cuda_headers} ${features_cuda_templates})
SOURCE_GROUP(features\\interface FILES ${features_interface_sources} ${features_interface_headers} ${features_interface_templates})
SOURCE_GROUP(features\\shared FILES ${features_shared_headers})
SOURCE_GROUP(keypoints FILES ${keypoints_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/grove)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/helpers)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/spaint/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
