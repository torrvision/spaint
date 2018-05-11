#! /bin/bash -e

PLATFORM=`../detect-platform.sh`

# Check whether a valid Vicon archive has been specified. If not, try a default if possible, or exit.
if [ $# -eq 1 ]
then
  archive=$1
else
  if [ $PLATFORM == "mac" ]
  then
    archive=~/Downloads/vicon/Vicon_DataStream_SDK_1.3_MAC.zip
  else
    echo "Usage: ./extract-vicon-nix.sh {Vicon archive}"
    exit
  fi
fi

echo "[spaint] Setting up Vicon DataStream SDK"

if [ -d vicon ]
then
  echo "[spaint] ...Skipping (already set up)"
  exit
else
  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir -p tmp/vicon-setup
  unzip $archive -d tmp/vicon-setup > /dev/null 2>& 1

  echo "[spaint] ...Normalising directory structure..."
  cd tmp
  mkdir -p vicon/include/vicon
  mkdir -p vicon/lib

  if [ $PLATFORM == "mac" ]
  then
    cp vicon-setup/Client.h vicon/include/vicon
    cp vicon-setup/lib* vicon/lib

    # Fix the library install names (prevents problems with finding the libraries at runtime).
    echo "[spaint] ...Fixing library install names..."
    cd vicon/lib
    ls | while read f
    do
      install_name_tool -id "$f" "$f"
    done
    install_name_tool -change "/Users/SWTest/jenkins/workspace/DatastreamSDK_Mac_Release_1.3/Binary/bin/Release/libDebugServices.dylib" "libDebugServices.dylib" libViconDataStreamSDK_CPP.dylib
    cd ../..
  else
    cp vicon-setup/ViconDataStreamSDK_1.4.0.67498/Linux64-boost-1.53.0/20140404_67498h/Release/Client.h vicon/include/vicon
    cp vicon-setup/ViconDataStreamSDK_1.4.0.67498/Linux64-boost-1.53.0/20140404_67498h/Release/lib* vicon/lib
  fi

  # Move the vicon directory into libraries and get rid of the tmp directory.
  cd ..
  mv tmp/vicon .
  /bin/rm -fR tmp

  echo "[spaint] ...Finished setting up Vicon DataStream SDK."
fi
