#! /bin/bash -e

LOG=../build-boost_1_56_0.log

# Check that valid parameters have been specified.
if [ $# -ne 1 ] || ([ "$1" != "msvc-11.0" ] && [ "$1" != "msvc-12.0" ])
then
  echo "Usage: build-boost_1_56_0-win.sh {msvc-11.0|msvc-12.0}"
  exit 1
fi

echo "[spaint] Building Boost 1.56.0 for $1"

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
  cmd //c "bootstrap.bat > $LOG"
fi

echo "[spaint] ...Running build..."
cmd //c "b2 -j2 --libdir=..\boost_1_56_0\lib --includedir=..\boost_1_56_0\include --abbreviate-paths --with-chrono --with-date_time --with-filesystem --with-regex --with-serialization --with-test --with-thread --build-type=complete --layout=tagged toolset=$1 architecture=x86 address-model=64 install >> $LOG"

echo "[spaint] ...Finished building Boost 1.56.0."
