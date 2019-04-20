#!/usr/bin/gnuplot -persist

delta_v(x) = (vD = x - old_v, old_v = x, vD)
old_v = NaN

set datafile separator ','
set autoscale fix
set title 'Y Axis Delta data'
set key autotitle columnhead
plot '/home/lbarnett/development/slam2/build/outputs.csv' using 18:($1), '' using 18:(delta_v($1)) title 'y Delta' with lines



