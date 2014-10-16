#! /bin/bash -e

LOG=../../build-SDL2-2.0.3.log

# Check that valid parameters have been specified.
if [ $# -ne 1 ] || ([ "$1" != "Visual Studio 11 Win64" ] && [ "$1" != "Visual Studio 12 Win64" ])
then
  echo "Usage: build-SDL2-2.0.3-win.sh {Visual Studio 11 Win64|Visual Studio 12 Win64}"
  exit 1
fi

# Build SDL.
echo "[spaint] Building SDL 2.0.3 for $1"

if [ -d SDL2-2.0.3 ]
then
  echo "[spaint] ...Skipping archive extraction (already extracted)"
else
  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp
  tar xzf ../setup/SDL2-2.0.3/SDL2-2.0.3.tar.gz
  cd ..
  mv tmp/SDL2-2.0.3 .
  rmdir tmp
fi

cd SDL2-2.0.3

if [ -d build ]
then
  echo "[spaint] ...Skipping build (already built)"
else
  mkdir build
  cd build

  echo "[spaint] ...Configuring using CMake..."
  cmake -DCMAKE_INSTALL_PREFIX=../install -DDIRECTX=OFF -G "$1" .. > $LOG 2>&1

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

echo "[spaint] ...Finished building SDL 2.0.3."
