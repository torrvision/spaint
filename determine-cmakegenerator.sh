#! /bin/bash -e

if [ $# -ne 1 ]
then
  echo "Usage: determine-cmakegenerator.sh {11|12|14|15}"
  exit
fi

CMAKE_GENERATOR="Visual Studio $1 Win64"

if [ "$1" == "15" ]
then
  CMAKE_GENERATOR="Visual Studio 15 2017 Win64"
fi

echo $CMAKE_GENERATOR
