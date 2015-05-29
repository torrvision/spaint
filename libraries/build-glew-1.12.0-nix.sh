#! /bin/bash -e

LOG=../build-glew-1.12.0.log

# Build glew.
echo "[spaint] Building glew 1.12.0"

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

cd glew-1.12.0

echo "[spaint] ...Running build..."
make -j2 >> $LOG 2>&1

cd ..

echo "[spaint] ...Finished building glew 1.12.0."
