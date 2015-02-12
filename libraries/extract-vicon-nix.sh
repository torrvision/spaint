#! /bin/bash -e

PLATFORM=`../detect-platform.sh`

# Check that a valid Vicon archive has been specified.
if [ $# -ne 1 ]
then
  echo "Usage: ./extract-vicon-nix.sh {Vicon archive}"
  exit
fi

echo "[spaint] Setting up Vicon DataStream SDK"

if [ -d vicon ]
then
  echo "[spaint] ...Skipping archive extraction (already extracted)"
  exit
else
  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir -p tmp/vicon-setup
  unzip $1 -d tmp/vicon-setup > /dev/null 2>& 1

  echo "[spaint] ...Normalising directory structure..."
  cd tmp
  mkdir -p vicon/include/vicon
  mkdir -p vicon/lib

  if [ $PLATFORM == "mac" ]
  then
    cp vicon-setup/Client.h vicon/include/vicon
    cp vicon-setup/lib* vicon/lib
  fi

  # Move the vicon directory into libraries and get rid of the tmp directory.
  cd ..
  mv tmp/vicon .
  /bin/rm -fR tmp

  echo "[spaint] ...Finished setting up Vicon DataStream SDK."
fi
