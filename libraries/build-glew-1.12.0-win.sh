#! /bin/bash -e

# Check that msbuild is on the system path.
../require-devenv.sh
../require-msbuild.sh

LOG=../../../build-glew-1.12.0.log

# Check that valid parameters have been specified.
if [ $# -ne 1 ] || ([ "$1" != "11" ] && [ "$1" != "12" ] && [ "$1" != "15" ])
then
  echo "Usage: build-glew-1.12.0-win.sh {11|12|15}"
  exit 1
fi

# Build glew.
echo "[spaint] Building glew 1.12.0 for Visual Studio $1 Win64"

if [ -d glew-1.12.0 ]
then
  echo "[spaint] ...Skipping archive extraction (already extracted)"
else
  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp
  tar xzf ../setup/glew-1.12.0/glew-1.12.0.tgz
  cd ..
  mv tmp/glew-1.12.0 .
  rmdir tmp
fi

cd glew-1.12.0/build/vc12

if [ $1 == "15" ]
then
  echo "[spaint] ...Upgrading solution..."
  cmd //c "devenv /upgrade glew.sln > $LOG 2>&1"
  result=`cmd //c "(vsdevcmd && set) | grep 'WindowsSDKVersion' | perl -pe 's/.*=(.*)./\1/g'"`
  ls *.vcxproj | while read f; do perl -ibak -pe 's/<ProjectGuid>\{(.*?)\}<\/ProjectGuid>/<ProjectGuid>\{\1\}<\/ProjectGuid>\r    <WindowsTargetPlatformVersion>'$result'<\/WindowsTargetPlatformVersion>/g' "$f"; perl -ibak -pe 's/v141/v140/g' "$f"; done
fi

echo "[spaint] ...Running build..."
cmd //c "msbuild /p:Configuration=Release /p:Platform=x64 glew.sln >> $LOG 2>&1"

cd ..

echo "[spaint] ...Finished building glew-1.12.0."
