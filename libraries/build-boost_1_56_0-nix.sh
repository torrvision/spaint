#! /bin/bash -e

PLATFORM=`../detect-platform.sh`

# Select the correct toolset based on whether this is being run on Linux or Mac OS X.
TOOLSET=gcc
if [ $PLATFORM == "mac" ]
then
  TOOLSET=darwin
fi

# Build Boost 1.56.0.
LOG=../build-boost_1_56_0.log

echo "[spaint] Building Boost 1.56.0"

if [ -d boost_1_56_0 ]
then
  echo "[spaint] ...Skipping build (already built)"
  exit
fi

if [ -d boost-setup ]
then
  echo "[spaint] ...Skipping archive extraction (already extracted)"
else
  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp
  tar xzf ../setup/boost_1_56_0/boost_1_56_0.tar.gz
  cd ..
  mv tmp/boost_1_56_0 boost-setup
  rmdir tmp
fi

cd boost-setup

if [ -e b2 ]
then
  echo "[spaint] ...Skipping bootstrapping (b2 already exists)"
else
  echo "[spaint] ...Bootstrapping..."
  ./bootstrap.sh > $LOG
fi

echo "[spaint] ...Running build..."
./b2 -j2 --libdir=../boost_1_56_0/lib --includedir=../boost_1_56_0/include --abbreviate-paths --with-chrono --with-date_time --with-filesystem --with-regex --with-thread --build-type=complete --layout=tagged toolset=$TOOLSET architecture=x86 address-model=64 install >> $LOG

echo "[spaint] ...Finished building Boost 1.56.0."
