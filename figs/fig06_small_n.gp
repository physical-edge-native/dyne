# 図6(代替) 補完測定の1台・4分割・8分割: 全324セル安定(判定のみ。誤差値は元データ消失)
set encoding utf8
set terminal pngcairo size 1200,330 font "Noto Sans JP,11"
set output 'fig06_small_n.png'
set multiplot layout 1,3 title '補完測定(432構成)の1台・4分割・8分割: 全セル安定(幅・反復・評価点の全組合せ)'
set xrange [0.5:6.5]; set yrange [0.5:3.5]
set xtics ('0.10' 1,'0.20' 2,'0.30' 3,'0.41' 4,'0.50' 5,'0.60' 6)
set ytics ('幅8' 1,'幅16' 2,'幅24' 3)
set xlabel 'r = αΔt/h^2'
set title '1台'
plot '-' using 1:2 with points pt 5 ps 6 lc rgb '#1a9850' notitle
1 1
1 2
1 3
2 1
2 2
2 3
3 1
3 2
3 3
4 1
4 2
4 3
5 1
5 2
5 3
6 1
6 2
6 3
e
set title '4分割'
replot
set title '8分割'
replot
unset multiplot
