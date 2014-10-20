#! /bin/bash -e

LOG=../../build-SDL2-2.0.3.log
PLATFORM=`../detect-platform.sh`

echo "[spaint] Building SDL 2.0.3"

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
  cmake -DCMAKE_INSTALL_PREFIX=../install .. > $LOG 2>&1

  echo "[spaint] ...Running build..."
  make -j2 >> $LOG 2>&1

  echo "[spaint] ...Finished building SDL 2.0.3."
fi
