#!/usr/bin/gnuplot -persist

set datafile separator ','
set autoscale fix
set title 'Y Axis data'
set key autotitle columnhead
plot '/home/lbarnett/development/odometry/poses/gt.csv' using 4:12 title 'x-z Axis GT' with lines



