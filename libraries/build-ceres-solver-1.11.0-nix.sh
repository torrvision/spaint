#! /bin/bash -e

LOG=../../build-ceres-solver-1.11.0.log
PLATFORM=`../detect-platform.sh`

echo "[spaint] Building Ceres Solver 1.11.0"

# Make sure that all needed dependencies are available.
if [ -d Eigen-3.2.2 ]
then
  echo "[spaint] ...Skipping Eigen-3.2.2 (already extracted)"
else
  echo "[spaint] ...Extracting Eigen-3.2.2..."
  ./extract-Eigen-3.2.2.sh > /dev/null 2>&1
  echo "[spaint] ...Finished extracting Eigen-3.2.2"
fi

if [ -d glog-0.3.4 ]
then
  echo "[spaint] ...Skipping glog-0.3.4 (already built)"
else
  echo "[spaint] ...Building glog-0.3.4..."
  ./build-glog-0.3.4-nix.sh > /dev/null 2>&1
  echo "[spaint] ...Finished building glog-0.3.4"
fi

# Build Ceres Solver itself.
if [ -d ceres-solver-1.11.0 ]
then
  echo "[spaint] ...Skipping archive extraction (already extracted)"
else
  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp
  tar xzf ../setup/ceres-solver-1.11.0/ceres-solver-1.11.0.tar.gz
  cd ..
  mv tmp/ceres-solver-1.11.0 .
  rmdir tmp
fi

cd ceres-solver-1.11.0

ROOT_DIR=`pwd`
INSTALL_DIR="$ROOT_DIR/install"

if [ -d build ]
then
  echo "[spaint] ...Skipping build (already built)"
else
  mkdir build
  cd build

  echo "[spaint] ...Configuring using CMake..."
  if [ $PLATFORM == "mac" ]
  then
    cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" -DEIGEN_INCLUDE_DIR=../../Eigen-3.2.2 -DGLOG_INCLUDE_DIR=../../glog-0.3.4/installed/include -DGLOG_LIBRARY=../../glog-0.3.4/installed/lib/libglog.dylib -DCMAKE_CXX_FLAGS="-stdlib=libstdc++" -DCMAKE_EXE_LINKER_FLAGS="-stdlib=libstdc++"  .. > $LOG 2>&1
  else
    cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" -DEIGEN_INCLUDE_DIR=../../Eigen-3.2.2 -DGLOG_INCLUDE_DIR=../../glog-0.3.4/installed/include -DGLOG_LIBRARY=../../glog-0.3.4/installed/lib/libglog.so .. > $LOG 2>&1
  fi

  echo "[spaint] ...Running build..."
  make -j2 >> $LOG 2>&1

  echo "[spaint] ...Installing..."
  make install >> $LOG 2>&1

  echo "[spaint] ...Finished building Ceres Solver 1.11.0."
fi
