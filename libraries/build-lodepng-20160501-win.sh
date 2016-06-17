#! /bin/bash -e

# Check that msbuild is on the system path.
../require-msbuild.sh

LOG=../../build-lodepng-20160501.log

# Check that valid parameters have been specified.
if [ $# -ne 1 ] || ([ "$1" != "Visual Studio 11 Win64" ] && [ "$1" != "Visual Studio 12 Win64" ])
then
  echo "Usage: build-lodepng-20160501-win.sh {Visual Studio 11 Win64|Visual Studio 12 Win64}"
  exit 1
fi

# Build LodePNG.
echo "[spaint] Building LodePNG 20160501 for $1"

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
  cmake -DCMAKE_INSTALL_PREFIX=../install -G "$1" .. > $LOG 2>&1

  echo "[spaint] ...Running Debug build..."
  cmd //c "msbuild /p:Configuration=Debug lodepng.sln >> $LOG 2>&1"

  echo "[spaint] ...Running Release build..."
  cmd //c "msbuild /p:Configuration=Release lodepng.sln >> $LOG 2>&1"

  cd ..
fi

echo "[spaint] ...Finished building LodePNG 20160501."
