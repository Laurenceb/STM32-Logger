set datafile separator ','
if(columns==5) plot filename using 1:2 with lines, filename using 1:3 with lines, filename using 1:4 with lines axis x1y2, filename using 1:5 with lines axis x1y2 ;else if(columns==4) plot filename using 1:2 with lines, filename using 1:3 with lines, filename using 1:4 with lines axis x1y2; else if(columns==3) plot filename using 1:2 with lines, filename using 1:3 with lines ;else plot filename using 1:2 with lines
set y2tics
set xlabel "Time (Seconds)"
set ylabel "PPG"
set y2label "Pressure (PSI), Temperature (\260C)"
pause -1
