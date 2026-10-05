# 図7 補完測定の16分割: 劣化・発散の件数(各組合せ r6×反復3=18セル中)
set encoding utf8
set terminal pngcairo size 900,520 font "Noto Sans JP,11"
set output 'fig07_counts.png'
set title '16分割の劣化・発散の件数(幅×評価点の取り方ごと。各18セル中)'
set ylabel '件数'
set yrange [0:20]
set xrange [0.4:6.6]
set xtics ('幅8・格子点のみ' 1,'幅8・中点も' 2,'幅16・格子点のみ' 3,'幅16・中点も' 4,'幅24・格子点のみ' 5,'幅24・中点も' 6) rotate by -20
set grid ytics
set style data histogram
set boxwidth 0.35
set style fill solid 0.85
set arrow from 0.4,18 to 6.6,18 nohead dt 2 lc rgb '#555555'
set label '全18セル' at 0.5,18.8 tc rgb '#555555'
plot 'fig07_counts.dat' using ($1-0.19):2 with boxes lc rgb '#f39c12' title '劣化',      ''                 using ($1+0.19):3 with boxes lc rgb '#c0392b' title '発散'
