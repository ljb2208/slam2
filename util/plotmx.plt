#!/usr/bin/gnuplot -persist

set datafile separator ','
set autoscale fix
set title 'Y Axis data'
set key autotitle columnhead
plot '/home/lbarnett/development/slam2/build/outputs.csv' using 33 title 'x Axis motion' with lines

