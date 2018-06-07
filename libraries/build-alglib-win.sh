#! /bin/bash -e

# Check that msbuild is on the system path.
../require-msbuild.sh

LOG=../../build-alglib.log

# Check that valid parameters have been specified.
if [ $# -ne 1 ] || ([ "$1" != "Visual Studio 11 Win64" ] && [ "$1" != "Visual Studio 12 Win64" ] && [ "$1" != "Visual Studio 15 2017 Win64" ])
then
  echo "Usage: build-alglib-win.sh {Visual Studio 11 Win64|Visual Studio 12 Win64|Visual Studio 15 2017 Win64}"
  exit 1
fi

# Build ALGLIB.
echo "[spaint] Building ALGLIB for $1"

if [ -d alglib ]
then
  echo "[spaint] ...Skipping archive extraction (already extracted)"
else
  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp
  tar xzf ../setup/alglib/alglib.tar.gz
  cd ..
  mv tmp/alglib .
  rmdir tmp
fi

cd alglib

if [ -d build ]
then
  echo "[spaint] ...Skipping build (already built)"
else
  mkdir build
  cd build

  echo "[spaint] ...Configuring using CMake..."
  cmake -DCMAKE_INSTALL_PREFIX=../install -G "$1" -T v140 .. > $LOG 2>&1

  echo "[spaint] ...Running Debug build..."
  cmd //c "msbuild /p:Configuration=Debug alglib.sln >> $LOG 2>&1"

  echo "[spaint] ...Running Release build..."
  cmd //c "msbuild /p:Configuration=Release alglib.sln >> $LOG 2>&1"

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

echo "[spaint] ...Finished building ALGLIB."
