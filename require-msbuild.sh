#! /bin/bash

result=`cmd //c msbuild 2>&1`
if [[ $result == *"not recognized"* ]]
then
  echo "Error: msbuild not found (you may need to add it to your system path)."
  exit 1
fi
