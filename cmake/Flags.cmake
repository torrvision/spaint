###############
# Flags.cmake #
###############

# If on Mac OS X 10.9 (Mavericks), make sure everything compiles and links using the correct C++ Standard Library.
IF(${CMAKE_SYSTEM} MATCHES "Darwin-13.")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
ENDIF()

# If on Linux, make sure that C++11 support is enabled.
IF(${CMAKE_SYSTEM} MATCHES "Linux")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

# If on Windows and using Visual Studio, make sure that the maths constants are defined.
IF(MSVC_IDE)
  ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
ENDIF()
