#! /bin/bash -e

/bin/rm -fR build
/bin/rm -fR docs
/bin/rm -fR install

cd libraries
/bin/rm -fR boost_1_56_0
/bin/rm -fR boost-setup
/bin/rm -fR Eigen-3.2.2
/bin/rm -fR opencv-3.1.0
/bin/rm -fR SDL2-2.0.3
/bin/rm -fR SDL2-2.0.7
/bin/rm -fR vicon
/bin/rm -fR *.log
