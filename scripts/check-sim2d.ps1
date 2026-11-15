# 2次元の土台 (dyne-sim2d + heat2d-actor) と格子型アクター (sdf2d-grid-actor) の手元確認スクリプト (2026-10-10)
# リポジトリのルートで実行する: powershell -ExecutionPolicy Bypass -File scripts\check-sim2d.ps1
#
# 確認内容と期待結果:
#   [1] ビルドと単体テスト … dyne-wire 6件 ok、アクターのwasmと dyne-sim2d がビルドできる
#   [2] ビット一致 … 9シナリオすべて "PASS (bit-exact)" (分割の形・外周の種類・
#       損失・遅延・一時停止・分断の組み合わせ)
#   [3] 決定論 … 同じ引数の2回実行で標準出力がバイト一致 (fc「相違点は検出されませんでした」)
#   [4] 2次精度 … 解析解との最大誤差が刻み半分で約1/4 (nx=21/41/81 で 6.975e-4 / 1.741e-4 / 4.350e-5)
#   [5] 格子型アクター … 10シナリオすべて "PASS (bit-exact)" (空の部屋・雑多な部屋、分割の形、
#       損失・遅延・一時停止・分断、対象なしを刻む設定、距離の定義、2026-11-13 追加: 箱を動かす場面
#       (room6_move・room6_carry) を推奨の忘れる仕組み + 損失・遅延で)。決定論 (fc 相違なし)
#   [6] 対照実験 … 統合の遅れを足りなくした (3×3で L=1) 実行が "FAIL" になる (遅着を検出できること)
#   [7] 2026-11-15 追加 (GitHub issue #26): 2×2 の輪で待ち合って止まらないこと。制限時間 (60秒) 内に
#       3行とも期待どおり (heat と首振りの場面は "PASS (bit-exact)"、全損失は終了コード2の "STALL")
# 既存の1次元回帰の無変化は scripts\check-baseline12.ps1 で別に確かめる。

$ErrorActionPreference = "Stop"
$root = (Get-Location).Path
$w = Join-Path $root "crates\heat2d-actor\target\wasm32-unknown-unknown\release\heat2d_actor.wasm"
$exe = Join-Path $root "target\release\dyne-sim2d.exe"

Write-Host "== [1] ビルドと単体テスト =="
cargo test -p dyne-wire --release
Push-Location crates\heat2d-actor
cargo build --release --target wasm32-unknown-unknown
Pop-Location
Push-Location crates\sdf2d-grid-actor
cargo test --release
cargo build --release --target wasm32-unknown-unknown
Pop-Location
cargo build --release -p dyne-sim2d

Write-Host "== [2] ビット一致 (9行とも PASS のこと) =="
$scenarios = [ordered]@{
    "basic2x2"      = @("--px","2","--py","2","--steps","500")
    "loss10_lat"    = @("--px","2","--py","2","--steps","500","--loss","0.1","--lat-ms","5","--lat-jitter-ms","3")
    "loss30_jitter" = @("--px","2","--py","2","--steps","500","--loss","0.3","--lat-ms","20","--lat-jitter-ms","30","--seed","7")
    "pause_part"    = @("--px","2","--py","2","--steps","500","--loss","0.1","--pause","3@2000:6000","--partition","0-1@10000:15000","--partition","1-3@12000:20000")
    "3x2_uneven"    = @("--px","3","--py","2","--nx","40","--ny","33","--steps","400","--loss","0.15","--lat-ms","5","--lat-jitter-ms","10")
    "1x4_strip"     = @("--px","1","--py","4","--steps","300","--loss","0.1")
    "4x4_pause"     = @("--px","4","--py","4","--nx","61","--ny","61","--steps","300","--dt","4e-5","--loss","0.1","--lat-ms","3","--lat-jitter-ms","3","--pause","5@3000:8000")
    "zero_grad"     = @("--px","2","--py","2","--steps","500","--outer-bc","zero_grad","--loss","0.1")
    "3x3_dir05"     = @("--px","3","--py","3","--nx","45","--ny","31","--steps","400","--outer-bc","dirichlet(0.5)","--loss","0.2","--lat-jitter-ms","50")
}
foreach ($k in $scenarios.Keys) {
    $line = & $exe --wasm $w --quiet @($scenarios[$k]) | Select-String -Pattern "PASS|FAIL"
    Write-Host ("{0,-14} {1}" -f $k, $line)
}

Write-Host "== [3] 決定論 (相違なしのこと) =="
$work = Join-Path $env:TEMP "dyne-sim2d-check"
if (Test-Path $work) { Remove-Item -Recurse -Force $work }
New-Item -ItemType Directory $work | Out-Null
$args3 = $scenarios["pause_part"]
& $exe --wasm $w @args3 > (Join-Path $work "a.txt")
& $exe --wasm $w @args3 > (Join-Path $work "b.txt")
fc.exe (Join-Path $work "a.txt") (Join-Path $work "b.txt")

Write-Host "== [4] 2次精度 (解析解との誤差が約1/4ずつ) =="
foreach ($n in 21, 41, 81) {
    $dt = 0.16 / (($n - 1) * ($n - 1))
    $steps = [math]::Round(0.05 / $dt)
    $line = & $exe --wasm $w --quiet --px 2 --py 2 --nx $n --ny $n --dt $dt --steps $steps | Select-String -Pattern "analytic"
    Write-Host ("nx={0,-3} {1}" -f $n, $line)
}

$ws = Join-Path $root "crates\sdf2d-grid-actor\target\wasm32-unknown-unknown\release\sdf2d_grid_actor.wasm"
$em = Join-Path $root "crates\dyne-sim2d\scenes\empty.txt"
$cl = Join-Path $root "crates\dyne-sim2d\scenes\cluttered.txt"

$mv = Join-Path $root "crates\dyne-sim2d\scenes\room6_move.txt"
$ca = Join-Path $root "crates\dyne-sim2d\scenes\room6_carry.txt"
# 推奨の忘れる仕組み (docs/2026-11-11-dynamic-shape.md) と、速くする工夫の設定 (docs/2026-11-11-shape-speedup.md)
$forget = @("--step-ms","66.667","--noise-m","0.008","--disagree","0.1","--decay","0.9","--w-forget","0.3")

Write-Host "== [5] 格子型アクター (10行とも PASS のこと) =="
$shape = [ordered]@{
    "empty_loss20"   = @("--scene",$em,"--steps","300","--loss","0.2","--lat-ms","10","--lat-jitter-ms","30")
    "empty_pause"    = @("--scene",$em,"--steps","300","--loss","0.1","--pause","1@2000:6000","--partition","0-2@5000:9000")
    "empty_3x3"      = @("--scene",$em,"--px","3","--py","3","--steps","300","--loss","0.1","--lat-jitter-ms","20")
    "empty_4x1"      = @("--scene",$em,"--px","4","--py","1","--steps","300","--loss","0.1")
    "clutter_2x2"    = @("--scene",$cl,"--steps","400")
    "clutter_3x2"    = @("--scene",$cl,"--px","3","--py","2","--steps","400","--loss","0.2","--lat-ms","5","--lat-jitter-ms","20","--pause","4@3000:7000")
    "clutter_carve"  = @("--scene",$cl,"--steps","400","--carve")
    "clutter_perp"   = @("--scene",$cl,"--steps","400","--range-def","perpendicular")
    # 2026-11-13 追加: 箱を動かす場面 (移動は 300 ステップ目)。4×4 分割・損失・遅延の下で、箱の移動の前後を通して
    # 分割解と単一ノード解がビット一致すること
    "move_4x4"       = @("--scene",$mv,"--px","4","--py","4","--steps","400","--loss","0.1","--lat-ms","5","--lat-jitter-ms","10") + $forget
    "carry_4x4_view" = @("--scene",$ca,"--px","4","--py","4","--steps","400","--loss","0.1","--lat-ms","5","--lat-jitter-ms","10","--sweep-every","2","--decay-in-view") + $forget
}
foreach ($k in $shape.Keys) {
    $line = & $exe --wasm $ws --quiet @($shape[$k]) | Select-String -Pattern "PASS|FAIL"
    Write-Host ("{0,-14} {1}" -f $k, $line)
}
$args5 = $shape["clutter_3x2"]
& $exe --wasm $ws @args5 > (Join-Path $work "c.txt")
& $exe --wasm $ws @args5 > (Join-Path $work "d.txt")
fc.exe (Join-Path $work "c.txt") (Join-Path $work "d.txt")

Write-Host "== [6] 対照実験 (FAIL になること) =="
& $exe --wasm $ws --quiet --scene $em --px 3 --py 3 --steps 200 --delay 1 | Select-String -Pattern "rays:|PASS|FAIL"

# 2026-11-15 追加 (GitHub issue #26): 2×2 の輪の4台が、それぞれ隣の1つからだけ受け取れていない状態で
# 同じステップに止まる (修正前は終わらなかった)。制限時間を超えたら打ち切って "TIMEOUT" と出す
Write-Host "== [7] 輪の待ちで止まらないこと (制限時間内に期待どおりのこと) =="
$pan = Join-Path $root "crates\dyne-sim2d\scenes\room6_move_pan4h.txt"
function Invoke-Limited([string]$name, [string[]]$argv, [int]$limitSec, [int]$wantExit, [string]$pattern) {
    $o = Join-Path $work "$name.out"
    $e = Join-Path $work "$name.err"
    $al = $argv | ForEach-Object { if ($_ -match '\s') { '"' + $_ + '"' } else { $_ } }
    $p = Start-Process -FilePath $exe -ArgumentList $al -NoNewWindow -PassThru -RedirectStandardOutput $o -RedirectStandardError $e
    $null = $p.Handle # 終了コードを読めるよう、終わる前にハンドルを取っておく (Windows PowerShell 5.1)
    if (-not $p.WaitForExit($limitSec * 1000)) {
        $p.Kill()
        Write-Host ("{0,-14} TIMEOUT (>{1} s)" -f $name, $limitSec)
        return
    }
    $p.WaitForExit()
    $line = Get-Content $o, $e | Select-String -Pattern $pattern | Select-Object -First 1
    $ok = ($p.ExitCode -eq $wantExit) -and $line
    Write-Host ("{0,-14} {1} (exit {2}) {3}" -f $name, $(if ($ok) { "OK" } else { "FAIL" }), $p.ExitCode, $line)
}
Invoke-Limited "ring_heat" @("--wasm",$w,"--quiet","--px","2","--py","2","--steps","200","--loss","0.3","--lat-jitter-ms","20") 60 0 "PASS \(bit-exact\)"
Invoke-Limited "ring_pan4h" @("--wasm",$ws,"--quiet","--scene",$pan,"--px","2","--py","2","--steps","200","--loss","0.3","--lat-jitter-ms","20") 60 0 "PASS \(bit-exact\)"
Invoke-Limited "stall_loss1" @("--wasm",$w,"--quiet","--px","2","--py","2","--steps","50","--loss","1.0") 60 2 "^STALL"
