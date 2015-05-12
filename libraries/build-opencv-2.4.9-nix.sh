#! /bin/bash -e

LOG=../../build-opencv-2.4.9.log
PLATFORM=`../detect-platform.sh`

echo "[spaint] Building OpenCV 2.4.9"

if [ $PLATFORM != "mac" ]
then
  pkg="libgtk2.0-dev"
  if dpkg --get-selections | grep -q "^$pkg[[:space:]]*install$" >/dev/null;
  then
    echo "[spaint] ...Checking if $pkg is installed... OK..."
  else
    echo "[spaint] $pkg is not installed... Exiting. Please type 'sudo apt-get install $pkg' and re-run this script."
    exit 1
  fi
fi

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
  if [ $PLATFORM == "mac" ]
  then
    echo "[spaint] ...Fixing FindCUDA.cmake..."
    perl -pi -e 's/^(?!#)(.*Wl.*)/#\1/' cmake/FindCUDA.cmake
  fi

  mkdir build
  cd build

  echo "[spaint] ...Configuring using CMake..."
  if [ $PLATFORM == "mac" ]
  then
    cmake -Wno-dev -DCMAKE_INSTALL_PREFIX=../install -DBUILD_DOCS=OFF -DBUILD_TESTS=OFF -DCMAKE_CXX_FLAGS="-stdlib=libstdc++" -DCMAKE_EXE_LINKER_FLAGS="-stdlib=libstdc++" -DWITH_CUDA=OFF .. > $LOG 2>&1
  else
    cmake -Wno-dev -DCMAKE_INSTALL_PREFIX=../install -DBUILD_DOCS=OFF -DBUILD_TESTS=OFF -DWITH_CUDA=OFF .. > $LOG 2>&1
  fi

  echo "[spaint] ...Running build..."
  make -j2 >> $LOG 2>&1

  echo "[spaint] ...Finished building OpenCV 2.4.9."
fi
