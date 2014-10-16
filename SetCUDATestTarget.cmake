###########################
# SetCUDATestTarget.cmake #
###########################

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/tests/${targetname})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin/tests/${targetname})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin/tests/${targetname})
CUDA_ADD_EXECUTABLE(${targetname} ${sources} ${headers} ${templates} OPTIONS --generate-code arch=compute_20,code=sm_20 --generate-code arch=compute_30,code=sm_30)
INCLUDE(${PROJECT_SOURCE_DIR}/VCLibraryHack.cmake)
