#!/usr/bin/gnuplot -persist

set datafile separator ','
set autoscale fix
set title 'Path Plot against Ground Truth'
set key autotitle columnhead
plot '/home/lbarnett/development/slam2/build/outputs.csv' using 17:19 title 'xz' with lines,  \
 '/home/lbarnett/development/odometry/poses/gt.csv' using 4:12 title 'xz GT' with lines


