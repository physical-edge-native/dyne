# 図1 主な掃引の安定地図(reference/learned_map.gp を復元データ用に調整)
set encoding utf8
set terminal pngcairo size 900,520 font "Noto Sans JP,11"
set output 'fig01_mapA.png'
set title '分割数と r ごとの安定性(中間層の幅16、反復5回、300ステップで判定)'
set xlabel 'r = αΔt/h^2'
set xrange [0.15:2.1]
set yrange [0.5:4.5]
set ytics ('1台' 1, '4分割' 2, '8分割' 3, '16分割' 4)
set grid xtics
set key outside right
set arrow from 0.5,0.5 to 0.5,4.5 nohead dt 2 lc rgb '#555555'
set label '差分法の上限 0.5' at 0.52,0.65 tc rgb '#555555'
set arrow from 0.7192,0.5 to 0.7192,4.5 nohead dt 4 lc rgb '#c0392b'
set label '理論の限界 0.719' at 0.74,4.35 tc rgb '#c0392b'
plot 'fig01_mapA.dat' using ($4==0?$2:1/0):5 with points pt 7 ps 1.6 lc rgb '#1a9850' title '安定',      ''               using ($4==1?$2:1/0):5 with points pt 9 ps 1.8 lc rgb '#f39c12' title '劣化',      ''               using ($4==2?$2:1/0):5 with points pt 2 ps 1.8 lw 2 lc rgb '#c0392b' title '発散'
