#! /bin/bash -e

LOG=../build-boost_1_58_0.log

# Check that valid parameters have been specified.
SCRIPT_NAME=`basename "$0"`

if [ $# -ne 1 ] || ([ "$1" != "11" ] && [ "$1" != "12" ] && [ "$1" != "14" ] && [ "$1" != "15" ])
then
  echo "Usage: $SCRIPT_NAME {11|12|14|15}"
  exit 1
fi

# Determine the Visual Studio toolset to use.
VS_TOOLSET="msvc-$1.0"

if [ "$1" == "15" ]
then
  VS_TOOLSET="msvc-14.0"
fi

# Build the library.
echo "[spaint] Building Boost 1.58.0 for $VS_TOOLSET"

if [ -d boost_1_58_0 ]
then
  echo "[spaint] ...Skipping build (already built)"
  exit
fi

if [ -d boost-setup ]
then
  echo "[spaint] ...Skipping archive extraction (already extracted)"
else
  mkdir -p setup/boost_1_58_0

  if [ ! -f setup/boost_1_58_0/boost_1_58_0.tar.gz ]
  then
    echo "[spaint] ...Downloading archive..."
    curl -sL http://sourceforge.net/projects/boost/files/boost/1.58.0/boost_1_58_0.tar.gz > setup/boost_1_58_0/boost_1_58_0.tar.gz
  fi

  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp
  tar xzf ../setup/boost_1_58_0/boost_1_58_0.tar.gz
  cd ..
  mv tmp/boost_1_58_0 boost-setup
  rmdir tmp
fi

cd boost-setup

if [ -e b2 ]
then
  echo "[spaint] ...Skipping bootstrapping (b2 already exists)"
else
  echo "[spaint] ...Bootstrapping..."
  cmd //c "bootstrap.bat > $LOG"
fi

echo "[spaint] ...Running build..."
cmd //c "b2 -j2 --libdir=..\boost_1_58_0\lib --includedir=..\boost_1_58_0\include --abbreviate-paths --with-chrono --with-date_time --with-filesystem --with-program_options --with-regex --with-serialization --with-test --with-thread --with-timer --build-type=complete --layout=tagged toolset=$VS_TOOLSET architecture=x86 address-model=64 install >> $LOG"

echo "[spaint] ...Fixing headers..."
perl -ibak -pe 's/#     pragma message\("Unknown compiler version/\/\/#     pragma message("Unknown compiler version/g' ../boost_1_58_0/include/boost/config/compiler/visualc.hpp
rm ../boost_1_58_0/include/boost/config/compiler/visualc.hppbak

echo "[spaint] ...Finished building Boost 1.58.0."
