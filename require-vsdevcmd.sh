#! /bin/bash

result=`cmd //c vsdevcmd 2>&1`
if [[ $result == *"not recognized"* ]]
then
  echo "Error: vsdevcmd not found. You probably need to add it to your system path."
  echo "The VS2017 version is generally in C:\Program Files (x86)\Microsoft Visual Studio\2017\{Community|Professional}\Common7\Tools."
  exit 1
fi
