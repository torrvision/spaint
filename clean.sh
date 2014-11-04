#! /bin/bash -e

/bin/rm -fR build
/bin/rm -fR docs
/bin/rm -fR install

cd libraries
/bin/rm -fR boost_1_56_0
/bin/rm -fR boost-setup
/bin/rm -fR opencv-2.4.9
/bin/rm -fR SDL2-2.0.3
/bin/rm -fR *.log
