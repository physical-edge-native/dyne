# 図5(代替) 補完測定の16分割: 判定の地図(安定セルの誤差値は元データ消失のため判定のみ)
set encoding utf8
set terminal pngcairo size 1200,650 font "Noto Sans JP,11"
set output 'fig05_n16.png'
set multiplot layout 2,3 title '補完測定(432構成)の16分割: 安定性の判定(緑=安定、橙▲=劣化、赤×=発散)'
set xrange [0.5:6.5]; set yrange [0.5:3.5]
set xtics ('0.10' 1,'0.20' 2,'0.30' 3,'0.41' 4,'0.50' 5,'0.60' 6)
set ytics ('幅8' 1,'幅16' 2,'幅24' 3)
set xlabel 'r = αΔt/h^2'
set palette defined (0 '#1a9850', 1 '#f39c12', 2 '#c0392b')
set cbrange [-0.5:2.5]
unset colorbox

set title '反復2回・格子点のみ'
plot 'fig05_n16.dat' using ($1==1 && $2==2 ? $3:1/0):4:5 with points pt 5 ps 6 palette notitle,      ''               using ($1==1 && $2==2 && $5==1 ? $3:1/0):4 with points pt 9 ps 1.6 lc rgb 'black' notitle,      ''               using ($1==1 && $2==2 && $5==2 ? $3:1/0):4 with points pt 2 ps 1.8 lw 2 lc rgb 'black' notitle
set title '反復5回・格子点のみ'
plot 'fig05_n16.dat' using ($1==1 && $2==5 ? $3:1/0):4:5 with points pt 5 ps 6 palette notitle,      ''               using ($1==1 && $2==5 && $5==1 ? $3:1/0):4 with points pt 9 ps 1.6 lc rgb 'black' notitle,      ''               using ($1==1 && $2==5 && $5==2 ? $3:1/0):4 with points pt 2 ps 1.8 lw 2 lc rgb 'black' notitle
set title '反復10回・格子点のみ'
plot 'fig05_n16.dat' using ($1==1 && $2==10 ? $3:1/0):4:5 with points pt 5 ps 6 palette notitle,      ''               using ($1==1 && $2==10 && $5==1 ? $3:1/0):4 with points pt 9 ps 1.6 lc rgb 'black' notitle,      ''               using ($1==1 && $2==10 && $5==2 ? $3:1/0):4 with points pt 2 ps 1.8 lw 2 lc rgb 'black' notitle
set title '反復2回・中点も使用'
plot 'fig05_n16.dat' using ($1==2 && $2==2 ? $3:1/0):4:5 with points pt 5 ps 6 palette notitle,      ''               using ($1==2 && $2==2 && $5==1 ? $3:1/0):4 with points pt 9 ps 1.6 lc rgb 'black' notitle,      ''               using ($1==2 && $2==2 && $5==2 ? $3:1/0):4 with points pt 2 ps 1.8 lw 2 lc rgb 'black' notitle
set title '反復5回・中点も使用'
plot 'fig05_n16.dat' using ($1==2 && $2==5 ? $3:1/0):4:5 with points pt 5 ps 6 palette notitle,      ''               using ($1==2 && $2==5 && $5==1 ? $3:1/0):4 with points pt 9 ps 1.6 lc rgb 'black' notitle,      ''               using ($1==2 && $2==5 && $5==2 ? $3:1/0):4 with points pt 2 ps 1.8 lw 2 lc rgb 'black' notitle
set title '反復10回・中点も使用'
plot 'fig05_n16.dat' using ($1==2 && $2==10 ? $3:1/0):4:5 with points pt 5 ps 6 palette notitle,      ''               using ($1==2 && $2==10 && $5==1 ? $3:1/0):4 with points pt 9 ps 1.6 lc rgb 'black' notitle,      ''               using ($1==2 && $2==10 && $5==2 ? $3:1/0):4 with points pt 2 ps 1.8 lw 2 lc rgb 'black' notitle
unset multiplot
