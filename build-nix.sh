#! /bin/bash -e

# Check that a valid build type has been specified.
if [ $# -ne 2 ] || ([ "$1" != "Unix Makefiles" ] && [ "$1" != "Eclipse CDT4 - Unix Makefiles" ] && [ "$1" != "Xcode" ]) || ([ $2 != "Debug" ] && [ $2 != "Release" ] && [ $2 != "RelWithDebInfo" ])
then
  echo "Usage: build-nix.sh {Unix Makefiles|Eclipse CDT4 - Unix Makefiles|Xcode} {Debug|Release|RelWithDebInfo}"
  exit
fi

# Detect whether this is being run on Linux or Mac OS X.
PLATFORM=linux
if [ "$(uname)" == "Darwin" ]
then
  PLATFORM=mac
fi

# Build/extract the libraries.
cd libraries

./build-alglib-nix.sh
./build-boost_1_56_0-nix.sh
./build-lodepng-20160501-nix.sh
./build-opencv-3.1.0-nix.sh
./build-SDL2-2.0.3-nix.sh
./extract-Eigen-3.2.2.sh

if [ $PLATFORM == "linux" ]
then
  ./build-glew-1.12.0-nix.sh
fi

cd ..

# Set the source and build directories for spaint itself.
SOURCE_DIR=`pwd`
BUILD_DIR="$SOURCE_DIR/build"

if [ "$1" == "Eclipse CDT4 - Unix Makefiles" ]
then
  # Eclipse doesn't like having the build directory inside the project: use a sibling directory instead.
  BUILD_DIR="$SOURCE_DIR/../spaint-build"
fi

# Build spaint.
echo "[spaint] Building spaint in $BUILD_DIR"

if [ ! -d $BUILD_DIR ]
then
  mkdir $BUILD_DIR
  cd $BUILD_DIR

  # Note: We need to configure twice to handle conditional building.
  echo "[spaint] ...Configuring using CMake..."
  cmake -G"$1" -DCMAKE_BUILD_TYPE=$2 $SOURCE_DIR
  cmake $SOURCE_DIR

  cd $SOURCE_DIR
fi

cd $BUILD_DIR

echo "[spaint] ...Running build..."
make -j2

echo "[spaint] ...Installing..."
make install

echo "[spaint] ...Finished building spaint."
