# 図9 初期値の乱数6通りの最大誤差(16分割・r=0.25・中点も使用)
set encoding utf8
set terminal pngcairo size 900,520 font "Noto Sans JP,11"
set output 'fig09_seeds.png'
set title '重みの初期値の乱数だけを変えた6本(16分割、r=0.25、幅16、反復5回、中点も使用)'
set xlabel '乱数の種(1=既定)'
set ylabel '最大誤差'
set logscale y
set format y '10^{%L}'
set yrange [1e-3:1e2]
set xrange [0.4:6.6]
set xtics 1,1,6
set grid ytics
set boxwidth 0.6
set style fill solid 0.85
set arrow from 0.4, first 10 to 6.6, first 10 nohead dt 3 lc rgb '#888888'
set label '発散の判定線 10' at 4.9,14 tc rgb '#888888'
set arrow from 0.4, first 0.1 to 6.6, first 0.1 nohead dt 3 lc rgb '#bbbbbb'
set label '劣化の判定線 0.1' at 4.9,0.14 tc rgb '#bbbbbb'
plot 'fig09_seeds.dat' using ($3==0?$1:1/0):2 with boxes lc rgb '#1a9850' title '安定',      ''                using ($3==1?$1:1/0):2 with boxes lc rgb '#f39c12' title '劣化',      ''                using ($3==2?$1:1/0):2 with boxes lc rgb '#c0392b' title '発散'
