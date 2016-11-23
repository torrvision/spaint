#! /bin/bash -e

PLATFORM=`../detect-platform.sh`

TOOLSET=gcc
if [ $PLATFORM == "mac" ]
then
  TOOLSET=darwin
  OSXVERSION=`../detect-osxversion.sh`
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
if [ $PLATFORM == "mac" ] && [ "$OSXVERSION" -ge 13 ]
then
  STDLIBFLAGS='cxxflags="-stdlib=libstdc++" linkflags="-stdlib=libstdc++"'
else
  STDLIBFLAGS=''
fi

./b2 -j2 --libdir=../boost_1_56_0/lib --includedir=../boost_1_56_0/include --abbreviate-paths --with-chrono --with-date_time --with-filesystem --with-program_options --with-regex --with-serialization --with-test --with-thread --with-timer --build-type=complete --layout=tagged toolset=$TOOLSET architecture=x86 address-model=64 $STDLIBFLAGS install >> $LOG

echo "[spaint] ...Fixing headers..."
perl -i -pe 's/SPT<void>/SPT<const void>/g' ../boost_1_56_0/include/boost/serialization/shared_ptr_helper.hpp

if [ $PLATFORM == "mac" ]
then
  perl -i -pe 's/INT128__\)$/INT128__) && !defined(__CUDACC__)/g' ../boost_1_56_0/include/boost/config/compiler/clang.hpp
  perl -i -pe 's/~this_type\(\);$/~sp_counted_impl_pda<P, D, A>();/g' ../boost_1_56_0/include/boost/smart_ptr/detail/sp_counted_impl.hpp
fi

echo "[spaint] ...Finished building Boost 1.56.0."
