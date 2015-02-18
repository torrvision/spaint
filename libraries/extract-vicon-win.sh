#! /bin/bash -e

# Check that a valid Vicon directory has been specified.
if [ $# -ne 1 ]
then
  echo "Usage: ./extract-vicon-win.sh {Vicon directory}"
  exit
fi

echo "[spaint] Setting up Vicon DataStream SDK"

if [ -d vicon ]
then
  echo "[spaint] ...Skipping (already set up)"
  exit
else
  echo "[spaint] ...Making normalised directory structure..."
  /bin/rm -fR tmp
  mkdir -p tmp/vicon/include/vicon
  mkdir -p tmp/vicon/lib
  cp "$1/DataStream SDK/Win64/CPP/Client.h" tmp/vicon/include/vicon
  find "$1/DataStream SDK/Win64/CPP" -name *.dll | while read f; do cp "$f" tmp/vicon/lib; done
  find "$1/DataStream SDK/Win64/CPP" -name *.lib | while read f; do cp "$f" tmp/vicon/lib; done

  # Move the vicon directory into libraries and get rid of the tmp directory.
  mv tmp/vicon .
  /bin/rm -fR tmp

  echo "[spaint] ...Finished setting up Vicon DataStream SDK."
fi
