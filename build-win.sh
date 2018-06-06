#! /bin/bash -e

# Check that valid parameters have been specified.
if [ $# -ne 2 ] || ([ "$1" != "11" ] && [ "$1" != "12" ] && [ "$1" != "15" ]) || ([ "$2" != "Debug" ] && [ "$2" != "Release" ])
then
  echo "Usage: build-win.sh {11|12|15} {Debug|Release}"
  exit
fi

# Check that msbuild is on the system path.
./require-msbuild.sh

boosttoolset="msvc-$1.0"
cmakegenerator="Visual Studio $1 Win64"

if [ "$1" == 15 ]
then
  boosttoolset="msvc-14.0"
  cmakegenerator="Visual Studio 15 2017 Win64"
fi

cd libraries
./build-boost_1_67_0-win.sh "$boosttoolset"
./build-glew-1.12.0-win.sh $1
./build-lodepng-20160501-win.sh "$cmakegenerator"
#./build-opencv-3.1.0-win.sh "$cmakegenerator"
./build-SDL2-2.0.7-win.sh "$cmakegenerator"
./extract-Eigen-3.2.2.sh
cd ..

echo "[spaint] Building spaint"

if [ ! -d build ]
then
  mkdir build
  cd build

  # Note: We need to configure twice to handle conditional building.
  echo "[spaint] ...Configuring using CMake..."
  cmake -G "$cmakegenerator" -T v140 ..
  cmake ..

  cd ..
fi

cd build

echo "[spaint] ...Running build..."
cmd //c "msbuild /p:Configuration=$2 spaint.sln"

echo "[spaint] ...Installing..."
cmd //c "msbuild /p:Configuration=$2 INSTALL.vcxproj"

echo "[spaint] ...Finished building spaint."
