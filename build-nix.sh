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

./build-boost_1_56_0-nix.sh
./build-lodepng-20160501-nix.sh
#./build-opencv-3.1.0-nix.sh
./build-SDL2-2.0.3-nix.sh
./extract-Eigen-3.2.2.sh

if [ $PLATFORM == "linux" ]
then
  ./build-glew-1.12.0-nix.sh
fi

cd ..

# Set build and source folders
source_dir=`pwd`
build_dir="$source_dir/build"

if [ "$1" == "Eclipse CDT4 - Unix Makefiles" ]
then
  # Eclipse does not like having the build folder inside the project
  # Use a sibling instead
  build_dir="$source_dir/../spaint-build"
fi

# Build spaint itself.
echo "[spaint] Building spaint in $build_dir"

if [ ! -d $build_dir ]
then
  mkdir $build_dir
  cd $build_dir

  # Note: We need to configure twice to handle conditional building.
  echo "[spaint] ...Configuring using CMake..."
  cmake -G"$1" -DCMAKE_BUILD_TYPE=$2 $source_dir
  cmake $source_dir

  cd $source_dir
fi

cd $build_dir

echo "[spaint] ...Running build..."
make -j2

echo "[spaint] ...Installing..."
make install

echo "[spaint] ...Finished building spaint."
