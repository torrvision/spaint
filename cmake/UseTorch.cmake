##################
# UseTorch.cmake #
##################

OPTION(WITH_TORCH "Build with Torch support?" OFF)

IF(WITH_TORCH)
  IF(MSVC_IDE)
    SET(Caffe2_DIR "$ENV{HOMEDRIVE}/$ENV{HOMEPATH}/Downloads/libtorch/share/cmake/Caffe2" CACHE FILEPATH "The Caffe2 directory")
    SET(Torch_DIR "$ENV{HOMEDRIVE}/$ENV{HOMEPATH}/Downloads/libtorch/share/cmake/Torch" CACHE FILEPATH "The Torch directory")
  ELSEIF(${CMAKE_SYSTEM} MATCHES "Linux")
    SET(Caffe2_DIR "~/Downloads/libtorch/share/cmake/Caffe2" CACHE FILEPATH "The Caffe2 directory")
    SET(Torch_DIR "~/Downloads/libtorch/share/cmake/Torch" CACHE FILEPATH "The Torch directory")
  ENDIF()

  IF(POLICY CMP0002)
    CMAKE_POLICY(SET CMP0002 OLD)
  ENDIF()

  IF(NOT TARGET caffe2::cuda)
    FIND_PACKAGE(Torch REQUIRED)
  ENDIF()

  INCLUDE_DIRECTORIES(${TORCH_INCLUDE_DIRS})
  ADD_DEFINITIONS(-DWITH_TORCH)
ENDIF()
