plot '< tail -n 500 error.txt'  using 0:1 title 'CTE' with lines, '' using 0:3 title 'Steering' with lines
do for [t=0:300] {replot;pause 1;}   

