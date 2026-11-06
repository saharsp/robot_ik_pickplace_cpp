set terminal pngcairo size 900,800
set output 'ik_demo_result.png'
set xlabel 'X (m)'
set ylabel 'Y (m)'
set zlabel 'Z (m)'
set title 'From-scratch 6-DOF IK: pick-and-place configurations'
set view equal xyz
splot \
  'ik_demo_result.dat' index 0 with linespoints lw 2 pt 7 lc rgb 'gray' title 'Home', \
  'ik_demo_result.dat' index 1 with linespoints lw 2 pt 7 lc rgb 'blue' title 'Pick pose (IK solution)', \
  'ik_demo_result.dat' index 2 with linespoints lw 2 pt 7 lc rgb 'orange' title 'Place pose (IK solution)', \
  '-' with points pt 9 ps 2 lc rgb 'blue' title 'Pick target', \
  '-' with points pt 9 ps 2 lc rgb 'orange' title 'Place target'
0.5 0.3 0.4
e
-0.4 0.4 0.3
e
