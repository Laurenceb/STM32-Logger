#!usr/bin/bash
FILENAME=$(ls -t *.csv | head -1);
line=$(head $FILENAME -n 10 | tail -n 1)
echo $line
x=${line//[!,]/}
Y="${#x}"
echo $Y
echo $FILENAME
gnuplot -e "filename=\"$FILENAME\"" -e "columns=$Y" graph.plt;
