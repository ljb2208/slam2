#!/usr/bin/gnuplot -persist

set datafile separator ','
set autoscale fix
set title 'Y Axis data'
set key autotitle columnhead
plot '/home/lbarnett/development/slam2/build/outputs.csv' using 17:19 title 'xy' with lines,  \
 '/home/lbarnett/development/odometry/poses/00.csv' using 4:12 title 'xy GT' with lines


