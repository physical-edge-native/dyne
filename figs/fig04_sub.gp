# 図4 16分割: 評価点の取り方(格子点のみ / 中点も使用)の比較
set encoding utf8
set terminal pngcairo size 900,520 font "Noto Sans JP,11"
set output 'fig04_sub.png'
set title '16分割の最大誤差: 残差の評価点の取り方による違い(幅16、反復5回)'
set xlabel 'r = αΔt/h^2'
set ylabel '最大誤差'
set logscale y
set format y '10^{%L}'
set xrange [0.15:2.1]
set yrange [1e-4:1e4]
set grid
set key outside right
set arrow from graph 0, first 10 to graph 1, first 10 nohead dt 3 lc rgb '#888888'
set label '発散の判定線 10' at 1.5,20 tc rgb '#888888'
set label '発散は 10^3 の位置に×' at 0.2,3e3
plot 'fig04_sub.dat' index 0 using ($3<2?$1:1/0):2 with linespoints pt 7 lc rgb '#2c7bb6' title '格子点のみ',      ''              index 1 using ($3<2?$1:1/0):2 with linespoints pt 5 lc rgb '#8e44ad' title '格子の中点も使用',      ''              index 0 using ($3==2?$1:1/0):(1e3) with points pt 2 ps 1.6 lw 2 lc rgb '#2c7bb6' notitle,      ''              index 1 using ($3==2?$1:1/0):(1e3) with points pt 2 ps 1.6 lw 2 lc rgb '#8e44ad' notitle
