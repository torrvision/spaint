#! /bin/bash -e

PLATFORM=`../detect-platform.sh`

echo "[spaint] Setting up Vicon DataStream SDK"

if [ -d vicon ]
then
  echo "[spaint] ...Skipping archive extraction (already extracted)"
  exit
else
  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp
  mkdir vicon
  cd vicon
  unzip ../../setup/vicon/Vicon_DataStream_SDK_1.3_MAC.zip > /dev/null 2>&1
  cd ../..
  mv tmp/vicon .
  rmdir tmp
fi
