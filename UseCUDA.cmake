#################
# UseCUDA.cmake #
#################

FIND_PACKAGE(CUDA)

SET(CUDA_SEPARABLE_COMPILATION ON CACHE BOOL "" FORCE)

# If on Mac OS X 10.9 (Mavericks), make sure everything compiles and links using the correct C++ Standard Library.
IF(${CMAKE_SYSTEM} MATCHES "Darwin-13.")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")
	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
	SET(CUDA_HOST_COMPILER /usr/bin/clang)
	SET(CUDA_NVCC_FLAGS -Xcompiler -stdlib=libstdc++; -Xlinker -stdlib=libstdc++; ${CUDA_NVCC_FLAGS})
ENDIF()
