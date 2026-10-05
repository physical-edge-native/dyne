# 図8 1000ステップの追確認(300ステップとの比較。時系列データは残っていないため2点比較)
set encoding utf8
set terminal pngcairo size 900,520 font "Noto Sans JP,11"
set output 'fig08_long.png'
set title '劣化セルの最大誤差: 300ステップと1000ステップの比較(単調に成長=遅い増幅)'
set ylabel '最大誤差'
set logscale y
set format y '10^{%L}'
set yrange [1e-2:2e1]
set xrange [0.4:4.6]
set xtics ('r=0.80・8分割' 1,'r=1.00・4分割' 2,'r=0.60・16分割' 3,'r=0.25・16分割・中点も使用' 4) rotate by -15
set grid ytics
set boxwidth 0.35
set style fill solid 0.85
set arrow from 0.4, first 10 to 4.6, first 10 nohead dt 3 lc rgb '#888888'
set label '発散の判定線 10' at 0.5,13 tc rgb '#888888'
set arrow from 0.4, first 0.1 to 4.6, first 0.1 nohead dt 3 lc rgb '#bbbbbb'
set label '劣化の判定線 0.1' at 0.5,0.13 tc rgb '#bbbbbb'
plot 'fig08_long.dat' using ($1-0.19):2 with boxes lc rgb '#74add1' title '300ステップ',      ''               using ($1+0.19):3 with boxes lc rgb '#2c7bb6' title '1000ステップ'
