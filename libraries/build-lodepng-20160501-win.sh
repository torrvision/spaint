#! /bin/bash -e

# Check that msbuild is on the system path.
../require-msbuild.sh

LOG=../../build-lodepng-20160501.log

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

# Build LodePNG.
echo "[spaint] Building LodePNG 20160501 for $CMAKE_GENERATOR"

if [ -d lodepng-20160501 ]
then
  echo "[spaint] ...Skipping archive extraction (already extracted)"
else
  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp
  unzip ../setup/lodepng-20160501/lodepng-20160501.zip > /dev/null 2>&1
  cd ..
  mv tmp/lodepng-20160501 .
  rmdir tmp
fi

cd lodepng-20160501

if [ -d build ]
then
  echo "[spaint] ...Skipping build (already built)"
else
  mkdir build
  cd build

  echo "[spaint] ...Configuring using CMake..."
  cmake -DCMAKE_INSTALL_PREFIX=../install -G "$CMAKE_GENERATOR" $VS_TOOLSET_STRING .. > $LOG 2>&1

  echo "[spaint] ...Running Debug build..."
  cmd //c "msbuild /p:Configuration=Debug lodepng.sln >> $LOG 2>&1"

  echo "[spaint] ...Running Release build..."
  cmd //c "msbuild /p:Configuration=Release lodepng.sln >> $LOG 2>&1"

  cd ..
fi

echo "[spaint] ...Finished building LodePNG 20160501."
