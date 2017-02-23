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
SET(clustering_base_headers 
    include/grove/clustering/base/Mode3DColour.h
    include/grove/clustering/base/Prediction3DColour.h
    )
    
##
SET(clustering_cuda_headers include/grove/clustering/cuda/ExampleClusterer_CUDA.h)
SET(clustering_cuda_templates include/grove/clustering/cuda/ExampleClusterer_CUDA.tcu)
    
##
SET(clustering_interface_headers include/grove/clustering/interface/ExampleClusterer.h)
SET(clustering_interface_templates include/grove/clustering/interface/ExampleClusterer.tpp)

##
SET(features_sources src/features/FeatureCalculatorFactory.cpp)
SET(features_headers include/grove/features/FeatureCalculatorFactory.h)

##
SET(features_base_headers include/grove/features/base/Descriptor.h)

##
SET(features_cpu_headers include/grove/features/cpu/RGBDPatchFeatureCalculator_CPU.h)
SET(features_cpu_templates include/grove/features/cpu/RGBDPatchFeatureCalculator_CPU.tpp)

##
SET(features_cuda_headers include/grove/features/cuda/RGBDPatchFeatureCalculator_CUDA.h)
SET(features_cuda_templates include/grove/features/cuda/RGBDPatchFeatureCalculator_CUDA.tcu)

##
SET(features_interface_headers include/grove/features/interface/RGBDPatchFeatureCalculator.h)
SET(features_interface_templates include/grove/features/interface/RGBDPatchFeatureCalculator.tpp)

##
SET(features_shared_headers include/grove/features/shared/RGBDPatchFeatureCalculator_Shared.h)

##
SET(reservoirs_headers include/grove/reservoirs/ExampleReservoirsFactory.h)
SET(reservoirs_templates include/grove/reservoirs/ExampleReservoirsFactory.tpp)

##
SET(reservoirs_cpu_headers include/grove/reservoirs/cpu/ExampleReservoirs_CPU.h)
SET(reservoirs_cpu_templates include/grove/reservoirs/cpu/ExampleReservoirs_CPU.tpp)

##
SET(reservoirs_cuda_headers include/grove/reservoirs/cuda/ExampleReservoirs_CUDA.h)
SET(reservoirs_cuda_sources src/reservoirs/cuda/ExampleReservoirs_CUDA.cu)
SET(reservoirs_cuda_templates include/grove/reservoirs/cuda/ExampleReservoirs_CUDA.tcu)

##
SET(reservoirs_interface_headers include/grove/reservoirs/interface/ExampleReservoirs.h)
SET(reservoirs_interface_templates include/grove/reservoirs/interface/ExampleReservoirs.tpp)

##
SET(reservoirs_shared_headers include/grove/features/shared/RGBDPatchFeatureCalculator_Shared.h)

##
SET(keypoints_headers
include/grove/keypoints/Keypoint2D.h
include/grove/keypoints/Keypoint3DColour.h
)

##
SET(toplevel_sources src/CPUInstantiations.cpp)

IF(WITH_CUDA)
  SET(toplevel_sources ${toplevel_sources} src/CUDAInstantiations.cu)
ENDIF()

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${features_sources}
${toplevel_sources}
)

SET(headers
${clustering_base_headers}
${clustering_interface_headers}
${features_headers}
${features_base_headers}
${features_cpu_headers}
${features_interface_headers}
${features_shared_headers}
${reservoirs_headers}
${reservoirs_cpu_headers}
${reservoirs_interface_headers}
${reservoirs_shared_headers}
${keypoints_headers}
)

SET(templates
${clustering_interface_templates}
${features_cpu_templates}
${features_interface_templates}
${reservoirs_cpu_templates}
${reservoirs_interface_templates}
${reservoirs_templates}
)

IF(WITH_CUDA)
  SET(headers 
      ${headers}
      ${clustering_cuda_headers}
      ${features_cuda_headers}
      ${reservoirs_cuda_headers}
      )
      
  SET(sources
      ${sources}
      ${reservoirs_cuda_sources}
      )
      
  SET(templates
      ${templates}
      ${clustering_cuda_templates}
      ${features_cuda_templates}
      ${reservoirs_cuda_templates}
      )
ENDIF()

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${toplevel_sources})
SOURCE_GROUP(clustering\\base FILES ${clustering_base_headers})
SOURCE_GROUP(clustering\\cuda FILES ${clustering_cuda_headers} ${clustering_cuda_templates})
SOURCE_GROUP(clustering\\interface FILES ${clustering_interface_headers} ${clustering_interface_templates})
SOURCE_GROUP(features FILES ${features_sources} ${features_headers})
SOURCE_GROUP(features\\base FILES ${features_base_headers})
SOURCE_GROUP(features\\cpu FILES ${features_cpu_headers} ${features_cpu_templates})
SOURCE_GROUP(features\\cuda FILES ${features_cuda_headers} ${features_cuda_templates})
SOURCE_GROUP(features\\interface FILES ${features_interface_headers} ${features_interface_templates})
SOURCE_GROUP(features\\shared FILES ${features_shared_headers})
SOURCE_GROUP(reservoirs FILES ${reservoirs_sources} ${reservoirs_headers})
SOURCE_GROUP(reservoirs\\cpu FILES ${reservoirs_cpu_headers} ${reservoirs_cpu_templates})
SOURCE_GROUP(reservoirs\\cuda FILES ${reservoirs_cuda_headers} ${reservoirs_cuda_sources} ${reservoirs_cuda_templates})
SOURCE_GROUP(reservoirs\\interface FILES ${reservoirs_interface_headers} ${reservoirs_interface_templates})
SOURCE_GROUP(reservoirs\\shared FILES ${reservoirs_shared_headers})
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
