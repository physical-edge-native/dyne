# assets\*.wasm を wamrc (WAMR の事前コンパイラ) で .aot に変換する
#   使い方: powershell -ExecutionPolicy Bypass -File bench-c5\tools\make_aot.ps1 [-Target esp32c5|esp32] [-Wamrc 経路]
#
# wamrc は WAMR 2.4.0 (bench-c5 が使う espressif/wasm-micro-runtime と同じ版) を使うこと。
# AOT ファイルの形式は版に依存し、版が違うと実機の読み込みで拒否される。
# 入手方法は README.md の「AOT」の節を参照。
#
# 対象の指定:
#   ESP32-C5: RISC-V 32ビット、浮動小数点演算器なし (ESP-IDF の WAMR 設定も RISCV32_ILP32)
#             → --target=riscv32 --target-abi=ilp32 --cpu=generic-rv32 --cpu-features=+m,+a,+c
#   ESP32   : Xtensa → --target=xtensa
# --size-level=1 は、生成コードが大きなアドレス空間を前提にしない形にする指定 (マイコン向け)。
# --bounds-checks=1 は、線形メモリの範囲検査をコードに埋め込む (マイコンには
#   メモリ保護を使った検査の仕組みがないため必須)。
param(
    [ValidateSet("esp32c5", "esp32")] [string]$Target = "esp32c5",
    [string]$Wamrc = "wamrc"
)
$ErrorActionPreference = "Stop"
$assets = Join-Path (Split-Path -Parent $PSScriptRoot) "assets"

if (-not (Get-Command $Wamrc -ErrorAction SilentlyContinue)) {
    throw "wamrc が見つかりません ($Wamrc)。README.md の「AOT」の節の手順で入手し、-Wamrc で経路を渡してください"
}
if ($Target -eq "esp32c5") {
    $opt = @("--target=riscv32", "--target-abi=ilp32", "--cpu=generic-rv32", "--cpu-features=+m,+a,+c")
} else {
    $opt = @("--target=xtensa")
}
$opt += @("--size-level=1", "--bounds-checks=1")

foreach ($n in "mlp_actor_small", "heat1d_actor_small") {
    $in = Join-Path $assets "$n.wasm"
    $out = Join-Path $assets "$n.aot"
    & $Wamrc @opt -o $out $in
    if ($LASTEXITCODE -ne 0) { throw "wamrc が失敗しました: $n" }
    Write-Host ("{0}.aot: {1} バイト" -f $n, (Get-Item $out).Length)
}
Write-Host "注意: .aot は対象 ($Target) 専用。ターゲットを替えたら作り直すこと"
