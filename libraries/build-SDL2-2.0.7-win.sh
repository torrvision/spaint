#! /bin/bash -e

# Check that msbuild is on the system path.
../require-msbuild.sh

LOG=../../build-SDL2-2.0.7.log

# Check that valid parameters have been specified.
SCRIPT_NAME=`basename "$0"`

if [ $# -ne 1 ] || ([ "$1" != "11" ] && [ "$1" != "12" ] && [ "$1" != "14" ] && [ "$1" != "15" ])
then
  echo "Usage: $SCRIPT_NAME {11|12|14|15}"
  exit 1
fi

# Determine the CMake generator and Visual Studio toolset to use.
CMAKE_GENERATOR=`../determine-cmakegenerator.sh $1`
VS_TOOLSET_STRING=`../determine-vstoolsetstring.sh $1`

# Build SDL.
echo "[spaint] Building SDL 2.0.7 for $CMAKE_GENERATOR"

if [ -d SDL2-2.0.7 ]
then
  echo "[spaint] ...Skipping archive extraction (already extracted)"
else
  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp
  tar xzf ../setup/SDL2-2.0.7/SDL2-2.0.7.tar.gz
  cd ..
  mv tmp/SDL2-2.0.7 .
  rmdir tmp
fi

cd SDL2-2.0.7

if [ -d build ]
then
  echo "[spaint] ...Skipping build (already built)"
else
  mkdir build
  cd build

  echo "[spaint] ...Configuring using CMake..."
  cmake -DCMAKE_INSTALL_PREFIX=../install -DDIRECTX=OFF -G "$CMAKE_GENERATOR" $VS_TOOLSET_STRING .. > $LOG 2>&1

  echo "[spaint] ...Running build..."
  cmd //c "msbuild /p:Configuration=Debug SDL2.sln >> $LOG 2>&1"
  cmd //c "msbuild /p:Configuration=Release SDL2.sln >> $LOG 2>&1"

  cd ..
fi

if [ -d install ]
then
  echo "[spaint] ...Skipping install (already installed)"
else
  cd build

  echo "[spaint] ...Installing..."
  cmd //c "msbuild /p:Configuration=Debug INSTALL.vcxproj >> $LOG 2>&1"
  cmd //c "msbuild /p:Configuration=Release INSTALL.vcxproj >> $LOG 2>&1"

  cd ..
fi

echo "[spaint] ...Finished building SDL 2.0.7."
