#! /bin/bash -e

uname -r | perl -pe 's/(.*?)\..*/\1/'
