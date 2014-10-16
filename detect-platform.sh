#! /bin/bash -e

PLATFORM=linux
if [ "$(uname)" == "Darwin" ]
then
  PLATFORM=mac
fi

echo $PLATFORM
