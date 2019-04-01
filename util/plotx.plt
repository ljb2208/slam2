#!/usr/bin/gnuplot -persist

set datafile separator ','
set autoscale fix
set title 'X Axis data'
set key autotitle columnhead
plot '/home/lbarnett/development/slam2/build/outputs.csv' using 17 title 'x Axis' with lines,  \
 '/home/lbarnett/development/odometry/poses/00.csv' using 4 title 'x Axis GT' with lines



