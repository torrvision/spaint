#! /bin/bash -e

LOG=../../build-opencv-3.1.0.log
PLATFORM=`../detect-platform.sh`

echo "[spaint] Building OpenCV 3.1.0"

if [ $PLATFORM != "mac" ]
then
  ./install-dependencies-nix.sh libgtk2.0-dev
fi

if [ -d opencv-3.1.0 ]
then
  echo "[spaint] ...Skipping archive extraction (already extracted)"
else
  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp
  unzip ../setup/opencv-3.1.0/opencv-3.1.0.zip > /dev/null 2>&1
  cd ..
  mv tmp/opencv-3.1.0 .
  rmdir tmp
fi

cd opencv-3.1.0

if [ -d build ]
then
  echo "[spaint] ...Skipping build (already built)"
else
  mkdir -p install
  cd install
  INSTALL_PREFIX=`pwd`
  cd ..

  mkdir build
  cd build

  echo "[spaint] ...Configuring using CMake..."
  if [ $PLATFORM == "mac" ]
  then
    cmake -Wno-dev -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX -DBUILD_DOCS=OFF -DBUILD_TESTS=OFF -DCMAKE_CXX_FLAGS="-stdlib=libstdc++" -DCMAKE_EXE_LINKER_FLAGS="-stdlib=libstdc++" -DWITH_CUDA=OFF .. > $LOG 2>&1
  else
    cmake -Wno-dev -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX -DBUILD_DOCS=OFF -DBUILD_TESTS=OFF -DWITH_CUDA=OFF .. > $LOG 2>&1
  fi

  echo "[spaint] ...Running build..."
  make -j2 >> $LOG 2>&1

  echo "[spaint] ...Finished building OpenCV 3.1.0."
fi
