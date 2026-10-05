# 図3 掃引2(4分割、r=0.41)幅×反復回数
set encoding utf8
set terminal pngcairo size 900,520 font "Noto Sans JP,11"
set output 'fig03_r041.png'
set title '中間層の幅と反復回数ごとの最大誤差(4分割、r=0.41、300ステップ)'
set ylabel '最大誤差'
set logscale y
set format y '10^{%L}'
set yrange [1e-5:1e0]
set xrange [0:12]
set xtics ('反復2' 1, '反復5' 2, '反復10' 3, '反復2' 5, '反復5' 6, '反復10' 7, '反復2' 9, '反復5' 10, '反復10' 11)
set label '幅8' at 1.6,4e-1 center
set label '幅16' at 5.6,4e-1 center
set label '幅24' at 9.6,4e-1 center
set grid ytics
set boxwidth 0.7
set style fill solid 0.85
set arrow from 0, first 10 to 12, first 10 nohead dt 3 lc rgb '#888888'
set arrow from 0, first 0.1 to 12, first 0.1 nohead dt 3 lc rgb '#bbbbbb'
set key top left
plot 'fig03_r041.dat' using ($3==0?$1:1/0):2 with boxes lc rgb '#1a9850' title '安定',      ''                  using ($3==1?$1:1/0):2 with boxes lc rgb '#f39c12' title '劣化',      ''                  using ($3==2?$1:1/0):2 with boxes lc rgb '#c0392b' title '発散'
