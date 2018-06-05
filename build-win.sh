#! /bin/bash -e

# Check that valid parameters have been specified.
if [ $# -ne 2 ] || ([ "$1" != "11" ] && [ "$1" != "12" ] && [ "$1" != "15" ]) || ([ "$2" != "Debug" ] && [ "$2" != "Release" ])
then
  echo "Usage: build-win.sh {11|12|15} {Debug|Release}"
  exit
fi

# Check that msbuild is on the system path.
./require-msbuild.sh

cd libraries
if [ "$1" == 15 ]
then
  ./build-boost_1_67_0-win.sh "msvc-14.1"
else
  ./build-boost_1_56_0-win.sh "msvc-$1.0"
fi
./build-glew-1.12.0-win.sh $1
./build-lodepng-20160501-win.sh "Visual Studio $1 Win64"
#./build-opencv-3.1.0-win.sh "Visual Studio $1 Win64"
./build-SDL2-2.0.7-win.sh "Visual Studio $1 Win64"
./extract-Eigen-3.2.2.sh
cd ..

echo "[spaint] Building spaint"

if [ ! -d build ]
then
  mkdir build
  cd build

  # Note: We need to configure twice to handle conditional building.
  echo "[spaint] ...Configuring using CMake..."
  cmake -G "Visual Studio $1 Win64" ..
  cmake ..

  cd ..
fi

cd build

echo "[spaint] ...Running build..."
cmd //c "msbuild /p:Configuration=$2 spaint.sln"

echo "[spaint] ...Installing..."
cmd //c "msbuild /p:Configuration=$2 INSTALL.vcxproj"

echo "[spaint] ...Finished building spaint."
