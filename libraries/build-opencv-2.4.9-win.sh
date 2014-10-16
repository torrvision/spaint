#! /bin/bash -e

LOG=../../build-opencv-2.4.9.log

# Check that valid parameters have been specified.
if [ $# -ne 1 ] || ([ "$1" != "Visual Studio 11 Win64" ] && [ "$1" != "Visual Studio 12 Win64" ])
then
  echo "Usage: build-opencv-2.4.9-win.sh {Visual Studio 11 Win64|Visual Studio 12 Win64}"
  exit 1
fi

# Build OpenCV.
echo "[spaint] Building OpenCV 2.4.9 for $1"

if [ -d opencv-2.4.9 ]
then
  echo "[spaint] ...Skipping archive extraction (already extracted)"
else
  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp
  unzip ../setup/opencv-2.4.9/opencv-2.4.9.zip > /dev/null 2>&1
  cd ..
  mv tmp/opencv-2.4.9 .
  rmdir tmp
fi

cd opencv-2.4.9

if [ -d build ]
then
  echo "[spaint] ...Skipping build (already built)"
else
  mkdir build
  cd build

  echo "[spaint] ...Configuring using CMake..."
  cmake -Wno-dev -DCMAKE_INSTALL_PREFIX=../install -DBUILD_DOCS=OFF -DBUILD_TESTS=OFF -DBUILD_opencv_python=OFF -DWITH_CUDA=OFF -G "$1" .. > $LOG 2>&1

  echo "[spaint] ...Running Debug build..."
  cmd //c "msbuild /p:Configuration=Debug OpenCV.sln >> $LOG 2>&1"

  echo "[spaint] ...Running Release build..."
  cmd //c "msbuild /p:Configuration=Release OpenCV.sln >> $LOG 2>&1"

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

echo "[spaint] ...Finished building OpenCV 2.4.9."
