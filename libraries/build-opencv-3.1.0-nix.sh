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
  echo "[spaint] ...Extracting archives..."
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp
  unzip ../setup/opencv-3.1.0/opencv-3.1.0.zip > /dev/null 2>&1
  unzip ../setup/opencv-3.1.0/opencv_contrib-3.1.0.zip > /dev/null 2>&1
  cd ..
  mv tmp/opencv-3.1.0 .
  mv tmp/opencv_contrib-3.1.0 .
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
    CMAKE_CXX_FLAGS="-stdlib=libstdc++"
    CMAKE_EXE_LINKER_FLAGS="-stdlib=libstdc++"
  else
    CMAKE_CXX_FLAGS=""
    CMAKE_EXE_LINKER_FLAGS=""
  fi

  cmake -Wno-dev -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.1.0/modules -DBUILD_DOCS=OFF -DBUILD_TESTS=OFF -DWITH_CUDA=OFF -DCMAKE_CXX_FLAGS=$CMAKE_CXX_FLAGS -DCMAKE_EXE_LINKER_FLAGS=$CMAKE_EXE_LINKER_FLAGS -DBUILD_opencv_bgsegm=OFF -DBUILD_opencv_bioinspired=OFF -DBUILD_opencv_ccalib=OFF -DBUILD_opencv_datasets=OFF -DBUILD_opencv_dnn=OFF -DBUILD_opencv_dpm=OFF -DBUILD_opencv_face=OFF -DBUILD_opencv_fuzzy=OFF -DBUILD_opencv_line_descriptor=OFF -DBUILD_opencv_optflow=OFF -DBUILD_opencv_plot=OFF -DBUILD_opencv_python=OFF -DBUILD_opencv_reg=OFF -DBUILD_opencv_rgbd=OFF -DBUILD_opencv_saliency=OFF -DBUILD_opencv_stereo=OFF -DBUILD_opencv_structured_light=OFF -DBUILD_opencv_surface_matching=OFF -DBUILD_opencv_text=OFF -DBUILD_opencv_tracking=OFF -DBUILD_opencv_xfeatures2d=OFF -DBUILD_opencv_ximgproc=OFF -DBUILD_opencv_xobjdetect=OFF -DBUILD_opencv_xphoto=OFF .. > $LOG 2>&1

  echo "[spaint] ...Running build..."
  make -j2 >> $LOG 2>&1

  echo "[spaint] ...Finished building OpenCV 3.1.0."
fi
