#! /bin/bash -e

if [ $# -ne 1 ]
then
  echo "Usage: determine-vstoolsetstring.sh {11|12|14|15}"
  exit
fi

VS_TOOLSET_STRING=""

if [ "$1" == "15" ]
then
  VS_TOOLSET_STRING="-T v140"
fi

echo $VS_TOOLSET_STRING
