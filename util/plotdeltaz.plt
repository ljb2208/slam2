#!/usr/bin/gnuplot -persist

delta_v(x) = (vD = x - old_v, old_v = x, vD)
old_v = NaN

set datafile separator ','
set autoscale fix
set title 'Z Axis Delta data'
set key autotitle columnhead
plot '/home/lbarnett/development/slam2/build/outputs.csv' using 19:($1), '' using 19:(delta_v($1)) title 'Z Delta' with lines



