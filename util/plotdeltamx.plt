#!/usr/bin/gnuplot -persist

delta_v(x) = (vD = x - old_v, old_v = x, vD)
old_v = NaN

set datafile separator ','
set autoscale fix
set title 'X Axis Delta data'
set key autotitle columnhead
plot '/home/lbarnett/development/slam2/build/outputs.csv' using 34:($1), '' using 34:(delta_v($1)) title 'X Delta' with lines



