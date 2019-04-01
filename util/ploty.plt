#!/usr/bin/gnuplot -persist

set datafile separator ','
set autoscale fix
set title 'Y Axis data'
set key autotitle columnhead
plot '/home/lbarnett/development/slam2/build/outputs.csv' using 18 title 'y Axis' with lines,  \
 '/home/lbarnett/development/odometry/poses/00.csv' using 8 title 'y Axis GT' with lines



