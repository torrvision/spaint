####################################
# CMakeLists.txt for modules/grove #
####################################

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
SET(clustering_headers include/grove/clustering/ExampleClustererFactory.h)
SET(clustering_templates include/grove/clustering/ExampleClustererFactory.tpp)

##
SET(clustering_cpu_headers include/grove/clustering/cpu/ExampleClusterer_CPU.h)
SET(clustering_cpu_templates include/grove/clustering/cpu/ExampleClusterer_CPU.tpp)

##
SET(clustering_cuda_headers include/grove/clustering/cuda/ExampleClusterer_CUDA.h)
SET(clustering_cuda_templates include/grove/clustering/cuda/ExampleClusterer_CUDA.tcu)

##
SET(clustering_interface_headers include/grove/clustering/interface/ExampleClusterer.h)
SET(clustering_interface_templates include/grove/clustering/interface/ExampleClusterer.tpp)

##
SET(clustering_shared_headers include/grove/clustering/shared/ExampleClusterer_Shared.h)

##
SET(features_sources src/features/FeatureCalculatorFactory.cpp)
SET(features_headers include/grove/features/FeatureCalculatorFactory.h)
SET(features_templates include/grove/features/FeatureCalculatorFactory.tpp)

##
SET(features_base_headers
include/grove/features/base/Descriptor.h
include/grove/features/base/RGBDPatchFeatureDifferenceType.h
)

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
SET(forests_headers include/grove/forests/DecisionForestFactory.h)
SET(forests_templates include/grove/forests/DecisionForestFactory.tpp)

##
SET(forests_cpu_headers include/grove/forests/cpu/DecisionForest_CPU.h)
SET(forests_cpu_templates include/grove/forests/cpu/DecisionForest_CPU.tpp)

##
SET(forests_cuda_headers include/grove/forests/cuda/DecisionForest_CUDA.h)
SET(forests_cuda_templates include/grove/forests/cuda/DecisionForest_CUDA.tcu)

##
SET(forests_interface_headers include/grove/forests/interface/DecisionForest.h)
SET(forests_interface_templates include/grove/forests/interface/DecisionForest.tpp)

##
SET(forests_shared_headers include/grove/forests/shared/DecisionForest_Shared.h)

##
SET(keypoints_headers
include/grove/keypoints/Keypoint2D.h
include/grove/keypoints/Keypoint3DColour.h
)

##
SET(numbers_sources
src/numbers/CPURNG.cpp
)

SET(numbers_headers
include/grove/numbers/CPURNG.h
include/grove/numbers/CUDARNG.h
)

##
SET(reservoirs_headers include/grove/reservoirs/ExampleReservoirsFactory.h)
SET(reservoirs_templates include/grove/reservoirs/ExampleReservoirsFactory.tpp)

##
SET(reservoirs_cpu_headers include/grove/reservoirs/cpu/ExampleReservoirs_CPU.h)
SET(reservoirs_cpu_templates include/grove/reservoirs/cpu/ExampleReservoirs_CPU.tpp)

##
SET(reservoirs_cuda_headers include/grove/reservoirs/cuda/ExampleReservoirs_CUDA.h)
SET(reservoirs_cuda_templates include/grove/reservoirs/cuda/ExampleReservoirs_CUDA.tcu)

##
SET(reservoirs_interface_headers include/grove/reservoirs/interface/ExampleReservoirs.h)
SET(reservoirs_interface_templates include/grove/reservoirs/interface/ExampleReservoirs.tpp)

##
SET(reservoirs_shared_headers include/grove/reservoirs/shared/ExampleReservoirs_Shared.h)

##
SET(scoreforests_headers
include/grove/scoreforests/Keypoint3DColourCluster.h
)

##
SET(util_headers include/grove/util/Array.h)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${features_sources}
${numbers_sources}
)

SET(headers
${clustering_headers}
${clustering_cpu_headers}
${clustering_interface_headers}
${clustering_shared_headers}
${features_headers}
${features_base_headers}
${features_cpu_headers}
${features_interface_headers}
${features_shared_headers}
${forests_headers}
${forests_cpu_headers}
${forests_interface_headers}
${forests_shared_headers}
${keypoints_headers}
${numbers_headers}
${reservoirs_headers}
${reservoirs_cpu_headers}
${reservoirs_interface_headers}
${reservoirs_shared_headers}
${scoreforests_headers}
${util_headers}
)

SET(templates
${clustering_templates}
${clustering_cpu_templates}
${clustering_interface_templates}
${features_templates}
${features_cpu_templates}
${features_interface_templates}
${forests_templates}
${forests_cpu_templates}
${forests_interface_templates}
${reservoirs_templates}
${reservoirs_cpu_templates}
${reservoirs_interface_templates}
)

IF(WITH_CUDA)
  SET(headers ${headers}
    ${clustering_cuda_headers}
    ${features_cuda_headers}
    ${forests_cuda_headers}
    ${reservoirs_cuda_headers}
  )

  SET(templates ${templates}
    ${clustering_cuda_templates}
    ${features_cuda_templates}
    ${forests_cuda_templates}
    ${reservoirs_cuda_templates}
  )
ENDIF()

#############################
# Specify the source groups #
#############################

SOURCE_GROUP(clustering FILES ${clustering_headers} ${clustering_templates})
SOURCE_GROUP(clustering\\cpu FILES ${clustering_cpu_headers} ${clustering_cpu_templates})
SOURCE_GROUP(clustering\\cuda FILES ${clustering_cuda_headers} ${clustering_cuda_templates})
SOURCE_GROUP(clustering\\interface FILES ${clustering_interface_headers} ${clustering_interface_templates})
SOURCE_GROUP(clustering\\shared FILES ${clustering_shared_headers})
SOURCE_GROUP(features FILES ${features_sources} ${features_headers} ${features_templates})
SOURCE_GROUP(features\\base FILES ${features_base_headers})
SOURCE_GROUP(features\\cpu FILES ${features_cpu_headers} ${features_cpu_templates})
SOURCE_GROUP(features\\cuda FILES ${features_cuda_headers} ${features_cuda_templates})
SOURCE_GROUP(features\\interface FILES ${features_interface_headers} ${features_interface_templates})
SOURCE_GROUP(features\\shared FILES ${features_shared_headers})
SOURCE_GROUP(forests FILES ${forests_headers} ${forests_templates})
SOURCE_GROUP(forests\\cpu FILES ${forests_cpu_headers} ${forests_cpu_templates})
SOURCE_GROUP(forests\\cuda FILES ${forests_cuda_headers} ${forests_cuda_templates})
SOURCE_GROUP(forests\\interface FILES ${forests_interface_headers} ${forests_interface_templates})
SOURCE_GROUP(forests\\shared FILES ${forests_shared_headers})
SOURCE_GROUP(keypoints FILES ${keypoints_headers})
SOURCE_GROUP(numbers FILES ${numbers_sources} ${numbers_headers})
SOURCE_GROUP(reservoirs FILES ${reservoirs_sources} ${reservoirs_headers} ${reservoirs_templates})
SOURCE_GROUP(reservoirs\\cpu FILES ${reservoirs_cpu_headers} ${reservoirs_cpu_templates})
SOURCE_GROUP(reservoirs\\cuda FILES ${reservoirs_cuda_headers} ${reservoirs_cuda_templates})
SOURCE_GROUP(reservoirs\\interface FILES ${reservoirs_interface_headers} ${reservoirs_interface_templates})
SOURCE_GROUP(reservoirs\\shared FILES ${reservoirs_shared_headers})
SOURCE_GROUP(scoreforests FILES ${scoreforests_headers})
SOURCE_GROUP(util FILES ${util_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/grove)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/helpers)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/itmx/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
