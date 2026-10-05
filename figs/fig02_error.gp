# 図2 最大誤差と r(reference/learned_error.gp を復元データ用に調整)
set encoding utf8
set terminal pngcairo size 900,520 font "Noto Sans JP,11"
set output 'fig02_error.png'
set title '最大誤差(解析解との差)と r(中間層の幅16、反復5回、300ステップ)'
set xlabel 'r = αΔt/h^2'
set ylabel '最大誤差'
set logscale y
set format y '10^{%L}'
set xrange [0.15:2.1]
set yrange [1e-5:1e4]
set grid
set key outside right
set arrow from graph 0, first 10 to graph 1, first 10 nohead dt 3 lc rgb '#888888'
set label '発散の判定線 10' at 1.55,20 tc rgb '#888888'
set label '発散は 10^3 の位置に×(実測値は本文参照)' at 0.2,3e3
plot for [N in "1 4 8 16"] 'fig01_mapA.dat' using ($1==N && $4<2 ? $2:1/0):3 with linespoints pt 7 title N.(N eq "1" ? '台' : '分割'),      'fig01_mapA.dat' using ($4==2 ? $2:1/0):(1e3) with points pt 2 ps 1.6 lw 2 lc rgb '#c0392b' title '発散'
