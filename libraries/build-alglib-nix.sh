#! /bin/bash -e

LOG=../../build-alglib.log
PLATFORM=`../detect-platform.sh`

echo "[spaint] Building ALGLIB"

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
  if [ $PLATFORM == "mac" ]
  then
    cmake -DCMAKE_INSTALL_PREFIX=../install -DCMAKE_CXX_FLAGS="-stdlib=libstdc++" -DCMAKE_EXE_LINKER_FLAGS="-stdlib=libstdc++" .. > $LOG 2>&1
  else
    cmake -DCMAKE_INSTALL_PREFIX=../install .. > $LOG 2>&1
  fi

  echo "[spaint] ...Running build..."
  make -j2 >> $LOG 2>&1

  echo "[spaint] ...Installing..."
  make install >> $LOG 2>&1

  echo "[spaint] ...Finished building ALGLIB."
fi
