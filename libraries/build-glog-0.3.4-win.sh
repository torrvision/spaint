#! /bin/bash -e

# Check that devenv and msbuild are on the system path.
../require-devenv.sh
../require-msbuild.sh

LOG=../build-glog-0.3.4.log

# Check that valid parameters have been specified.
if [ $# -ne 1 ] || ([ "$1" != "Visual Studio 11 Win64" ] && [ "$1" != "Visual Studio 12 Win64" ])
then
  echo "Usage: build-glog-0.3.4-win.sh {Visual Studio 11 Win64|Visual Studio 12 Win64}"
  exit 1
fi

# Build glog.
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

if [ -d Debug ] || [ -d Release ]
then
  echo "[spaint] ...Skipping solution upgrade (already upgraded)"
else
  echo "[spaint] ...Upgrading Visual Studio solution..."
  cmd //c "devenv google-glog.sln /upgrade" > $LOG 2>&1

  echo "[spaint] ...Converting Visual Studio solution to x64..."
  perl -ibak -pe 's/Win32/x64/g' ./google-glog.sln

  find vsprojects -iname '*.vcxproj' | while read f
  do
    perl -ibak -pe 's/Win32/x64/g' $f
    perl -ibak -pe 's/MachineX86/MachineX64/g' $f
  done

  perl -ibak -pe 's/\_asm/\/\/_asm/g' ./src/logging.cc
fi

if [ -d Debug ]
then
  echo "[spaint] ...Skipping Debug build (already built)"
else
  echo "[spaint] ...Running Debug build..."
  cmd //c "msbuild /p:Configuration=Debug /p:Platform=x64 google-glog.sln >> $LOG 2>&1"
fi

if [ -d Release ]
then
  echo "[spaint] ...Skipping Release build (already built)"
else
  echo "[spaint] ...Running Release build..."
  cmd //c "msbuild /p:Configuration=Release /p:Platform=x64 google-glog.sln >> $LOG 2>&1"
fi

cd ..

echo "[spaint] ...Finished building glog 0.3.4."
