#!/usr/bin/gnuplot -persist

set datafile separator ','
set autoscale fix
set title 'Z Axis data'
set key autotitle columnhead
plot '/home/lbarnett/development/slam2/build/outputs.csv' using 19 title 'z Axis' with lines,  \
 '/home/lbarnett/development/odometry/poses/gt.csv' using 12 title 'z Axis GT' with lines



