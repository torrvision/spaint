#################
# UseCUDA.cmake #
#################

OPTION(WITH_CUDA "Build with CUDA support?" ON)

IF(WITH_CUDA)
  FIND_PACKAGE(CUDA)

  SET(CUDA_SEPARABLE_COMPILATION ON CACHE BOOL "" FORCE)

  # If on Windows, make it possible to enable GPU debug information.
  IF(MSVC_IDE)
    OPTION(ENABLE_CUDA_DEBUGGING "Enable CUDA debugging?" OFF)
    IF(ENABLE_CUDA_DEBUGGING)
      SET(CUDA_NVCC_FLAGS -G; ${CUDA_NVCC_FLAGS})
    ENDIF()
  ENDIF()

  # If on Mac OS X 10.9 (Mavericks), make sure everything compiles and links using the correct C++ Standard Library.
  IF(${CMAKE_SYSTEM} MATCHES "Darwin-13.")
    SET(CUDA_HOST_COMPILER /usr/bin/clang)
    SET(CUDA_NVCC_FLAGS -Xcompiler -stdlib=libstdc++; -Xlinker -stdlib=libstdc++; ${CUDA_NVCC_FLAGS})
  ENDIF()

  # If on Linux, make sure that C++11 support is enabled when compiling with nvcc.
  IF(${CMAKE_SYSTEM} MATCHES "Linux")
    SET(CUDA_NVCC_FLAGS -std=c++11; -Xcudafe "--diag_suppress=cc_clobber_ignored"; -Xcudafe "--diag_suppress=set_but_not_used"; ${CUDA_NVCC_FLAGS})
  ENDIF()

  ADD_DEFINITIONS(-DWITH_CUDA)
ENDIF()
