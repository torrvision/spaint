#! /bin/bash -e

echo "[spaint] Extracting Eigen 3.2.2"

if [ -d Eigen-3.2.2 ]
then
  echo "[spaint] ...Skipping archive extraction (already extracted)"
  exit
else
  echo "[spaint] ...Extracting archive..."
  /bin/rm -fR tmp
  mkdir tmp
  cd tmp
  tar xzf ../setup/Eigen-3.2.2/Eigen-3.2.2.tar.gz
  cd ..
  mv tmp/Eigen-3.2.2 .
  rmdir tmp
fi
