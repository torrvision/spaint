#! /bin/bash -e

LOG=../build-glog-0.3.4.log

# Build glew.
echo "[spaint] Building glog 0.3.4"

if [ -d glog-0.3.4 ]
then
  echo "[spaint] ...Skipping archive extraction (already extracted)"
else
  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp
  unzip ../setup/glog-0.3.4/glog-0.3.4.zip >> $LOG 2>&1
  cd ..
  mv tmp/glog-0.3.4 .
  rmdir tmp
fi

cd glog-0.3.4

echo "[spaint] ...Configuring..."
ROOT_DIR=`pwd`
INSTALL_DIR="$ROOT_DIR/installed"
./configure --prefix="$INSTALL_DIR" >> $LOG 2>&1

echo "[spaint] ...Running build..."
make -j2 >> $LOG 2>&1

echo "[spaint] ...Installing..."
make install >> $LOG 2>&1

cd ..

echo "[spaint] ...Finished building glog 0.3.4."
