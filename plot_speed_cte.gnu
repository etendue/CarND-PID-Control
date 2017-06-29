plot '< tail -n 500 error.txt'  using 0:2 title 'Speed Error' with lines, '' using 0:4 title 'Throttle' with lines
do for [t=0:300] {replot;pause 1;}   

