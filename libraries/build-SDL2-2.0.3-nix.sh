#! /bin/bash -e

LOG=../../build-SDL2-2.0.3.log
PLATFORM=`../detect-platform.sh`

echo "[corker] Building SDL 2.0.3"

if [ -d SDL2-2.0.3 ]
then
  echo "[corker] ...Skipping archive extraction (already extracted)"
else
  echo "[corker] ...Extracting archive..."
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
  echo "[corker] ...Skipping build (already built)"
else
  mkdir build
  cd build

  echo "[corker] ...Configuring using CMake..."
  cmake -DCMAKE_INSTALL_PREFIX=../install .. > $LOG 2>&1

  echo "[corker] ...Running build..."
  make -j2 >> $LOG 2>&1

  echo "[corker] ...Finished building SDL 2.0.3."
fi
