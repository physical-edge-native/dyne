# 図10 (reference/learned_width.gp のフォント・パスのみ調整)
# 4分割での発散境界の区間と中間層の幅
set encoding utf8
set terminal pngcairo size 800,500 font "Noto Sans JP,11"
set output 'fig10_width.png'
set title '発散の境界と中間層の幅(4分割、反復5回)'
set xlabel '中間層の幅'
set ylabel '発散の境界の r(区間で表示)'
set xrange [4:36]
set yrange [0.3:1.8]
set xtics (8,16,24,32)
set grid ytics
set key top right
set arrow from 4,0.7192 to 36,0.7192 nohead dt 4 lc rgb '#c0392b'
set label '理論の限界 0.719' at 26.5,0.67 tc rgb '#c0392b'
set arrow from 4,0.5 to 36,0.5 nohead dt 2 lc rgb '#555555'
set label '差分法の上限 0.5' at 26.5,0.46 tc rgb '#555555'
set arrow from 8,1.0 to 8,1.6 lw 2 lc rgb '#2c7bb6'
set label '上端は未測定' at 9,1.55
plot 'fig10_width.dat' using 1:($3==$3 ? $2:1/0):(0):($3-$2) with vectors nohead lw 6 lc rgb '#2c7bb6' title '境界がある区間', \
     'fig10_width.dat'                  using 1:2 with points pt 7 ps 1.2 lc rgb '#2c7bb6' notitle
