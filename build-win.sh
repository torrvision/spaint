#! /bin/bash -e

# Check that valid parameters have been specified.
if [ $# -ne 2 ] || ([ "$1" != "11" ] && [ "$1" != "12" ] && [ "$1" != "14" ] && [ "$1" != "15" ]) || ([ "$2" != "Debug" ] && [ "$2" != "Release" ])
then
  echo "Usage: build-win.sh {11|12|14|15} {Debug|Release}"
  exit
fi

# Check that msbuild is on the system path.
./require-msbuild.sh

# Build the third-party libraries.
cd libraries
./build-boost_1_58_0-win.sh "$1"
./build-glew-1.12.0-win.sh "$1"
./build-lodepng-20160501-win.sh "$1"
#./build-opencv-3.1.0-win.sh "$1"
./build-SDL2-2.0.7-win.sh "$1"
./extract-Eigen-3.2.2.sh
cd ..

# Build spaint itself.
echo "[spaint] Building spaint"

if [ ! -d build ]
then
  mkdir build
  cd build

  # Note: We need to configure twice to handle conditional building.
  echo "[spaint] ...Configuring using CMake..."
  CMAKE_GENERATOR=`../determine-cmakegenerator.sh $1`
  VS_TOOLSET_STRING=`../determine-vstoolsetstring.sh $1`
  cmake -G "$CMAKE_GENERATOR" $VS_TOOLSET_STRING ..
  cmake ..

  cd ..
fi

cd build

echo "[spaint] ...Running build..."
cmd //c "msbuild /p:Configuration=$2 spaint.sln"

echo "[spaint] ...Installing..."
cmd //c "msbuild /p:Configuration=$2 INSTALL.vcxproj"

echo "[spaint] ...Finished building spaint."
