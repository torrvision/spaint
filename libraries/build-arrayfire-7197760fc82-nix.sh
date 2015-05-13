#! /bin/bash -e

LOG=../../build-arrayfire-7197760fc82.log
PLATFORM=`../detect-platform.sh`

echo "[spaint] Building Arrayfire 7197760fc82"
./install-dependencies-nix.sh libatlas3gf-base libatlas-dev libfftw3-dev

if [ -d arrayfire-7197760fc82 ]
then
  echo "[spaint] ...Skipping archive extraction (already extracted)"
else
  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp
  unzip ../setup/arrayfire-7197760fc82/arrayfire-7197760fc82.zip > /dev/null 2>&1
  cd ..
  mv tmp/arrayfire-7197760fc82 .
  rmdir tmp
fi

cd arrayfire-7197760fc82

if [ -d build ]
then
  echo "[spaint] ...Skipping build (already built)"
else
  mkdir build
  cd build

  echo "[spaint] ...Configuring using CMake..."
  cmake -Wno-dev -DCMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=RELEASE -DBUILD_CUDA=ON .. > $LOG 2>&1

  echo "[spaint] ...Running build..."
  make -j8 >> $LOG 2>&1

  echo "[spaint] ...Finished building Arrayfire-7197760fc82."
fi
