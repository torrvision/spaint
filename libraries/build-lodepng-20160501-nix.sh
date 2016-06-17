#! /bin/bash -e

LOG=../../build-lodepng-20160501.log
PLATFORM=`../detect-platform.sh`

echo "[spaint] Building LodePNG 20160501"

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
  if [ $PLATFORM == "mac" ]
  then
    cmake -Wno-dev -DCMAKE_INSTALL_PREFIX=.. -DCMAKE_CXX_FLAGS="-stdlib=libstdc++" -DCMAKE_EXE_LINKER_FLAGS="-stdlib=libstdc++" .. > $LOG 2>&1
  else
    cmake -Wno-dev -DCMAKE_INSTALL_PREFIX=.. .. > $LOG 2>&1
  fi

  echo "[spaint] ...Running build..."
  make -j2 >> $LOG 2>&1

  echo "[spaint] ...Installing..."
  make install

  echo "[spaint] ...Finished building LodePNG 20160501."
fi
