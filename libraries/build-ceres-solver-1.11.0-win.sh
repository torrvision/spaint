#! /bin/bash -e

# Check that msbuild is on the system path.
../require-msbuild.sh

LOG=../../build-ceres-solver-1.11.0.log

# Check that valid parameters have been specified.
if [ $# -ne 1 ] || ([ "$1" != "Visual Studio 11 Win64" ] && [ "$1" != "Visual Studio 12 Win64" ])
then
  echo "Usage: build-ceres-solver-1.11.0-win.sh {Visual Studio 11 Win64|Visual Studio 12 Win64}"
  exit 1
fi

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
  ./build-glog-0.3.4-win.sh "$1" > /dev/null 2>&1
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
  cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" -DEIGEN_INCLUDE_DIR=../../Eigen-3.2.2 -DGLOG_INCLUDE_DIR=../../glog-0.3.4/src/windows -DGLOG_LIBRARY=../../glog-0.3.4/Debug/libglog.lib -G "$1" .. > $LOG 2>&1

  echo "[spaint] ...Running Debug build..."
  cmd //c "msbuild /p:Configuration=Debug /p:Platform=x64 Ceres.sln >> $LOG 2>&1"

  echo "[spaint] ...Configuring using CMake..."
  cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" -DEIGEN_INCLUDE_DIR=../../Eigen-3.2.2 -DGLOG_INCLUDE_DIR=../../glog-0.3.4/src/windows -DGLOG_LIBRARY=../../glog-0.3.4/Release/libglog.lib -G "$1" .. > $LOG 2>&1

  echo "[spaint] ...Running Release build..."
  cmd //c "msbuild /p:Configuration=Release /p:Platform=x64 Ceres.sln >> $LOG 2>&1"

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

echo "[spaint] ...Finished building Ceres Solver 1.11.0."
