#!/usr/bin/gnuplot -persist

set datafile separator ','
set autoscale fix
set title 'Feature Age Histogram'
set key autotitle columnhead
plot '/home/lbarnett/development/slam2/build/outputs.csv' using 42 title '0' with lines,  \
    '/home/lbarnett/development/slam2/build/outputs.csv' using 43 title '1' with lines,  \
    '/home/lbarnett/development/slam2/build/outputs.csv' using 44 title '2' with lines,  \
    '/home/lbarnett/development/slam2/build/outputs.csv' using 45 title '3' with lines,  \
    '/home/lbarnett/development/slam2/build/outputs.csv' using 46 title '4' with lines,  \
    '/home/lbarnett/development/slam2/build/outputs.csv' using 47 title '5' with lines,  \
    '/home/lbarnett/development/slam2/build/outputs.csv' using 48 title '6' with lines,  \
    '/home/lbarnett/development/slam2/build/outputs.csv' using 49 title '7' with lines,  \
    '/home/lbarnett/development/slam2/build/outputs.csv' using 50 title '8' with lines,  \
    '/home/lbarnett/development/slam2/build/outputs.csv' using 51 title '9' with lines,  \
    '/home/lbarnett/development/slam2/build/outputs.csv' using 52 title '10+' with lines
 

