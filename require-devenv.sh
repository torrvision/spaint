#! /bin/bash

result=`cmd //c devenv /? 2>&1`
if [[ $result == *"not recognized"* ]]
then
  echo "Error: devenv not found. You probably need to add it to your system path."
  echo "The VS2013 version is generally in C:\Program Files (x86)\Microsoft Visual Studio 12.0\Common7\IDE."
  exit 1
fi
