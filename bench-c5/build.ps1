# bench-c5 のビルド (実行方式とターゲットごとに別のビルドディレクトリを使う)
#   使い方: powershell -ExecutionPolicy Bypass -File bench-c5\build.ps1 [-Target esp32c5|esp32|esp32s3] [-Mode classic|fast|aot] [-Flash COM5]
#   例:     powershell -ExecutionPolicy Bypass -File bench-c5\build.ps1 -Target esp32c5 -Mode classic
#   -Flash を付けると、ビルド後にそのポートへ書き込んでモニタを開く (実機作業)
param(
    [ValidateSet("esp32c5", "esp32", "esp32s3")] [string]$Target = "esp32c5",
    [ValidateSet("classic", "fast", "aot")] [string]$Mode = "classic",
    [string]$Flash = ""
)
$ErrorActionPreference = "Stop"
$here = $PSScriptRoot
. C:\Espressif\Initialize-Idf.ps1 -IdfId esp-idf-7a8737a10725bc5cb19d516a1186b39f

$defaults = "sdkconfig.defaults"
if ($Mode -eq "fast") { $defaults += ";sdkconfig.fast" }
if ($Mode -eq "aot") {
    $defaults += ";sdkconfig.aot"
    foreach ($f in "mlp_actor_small.aot", "heat1d_actor_small.aot") {
        if (-not (Test-Path (Join-Path $here "assets\$f"))) {
            throw "assets\$f がありません。先に tools\make_aot.ps1 -Target $Target を実行してください"
        }
    }
}
$bdir = "build-$Target-$Mode"

Push-Location $here
try {
    # sdkconfig はビルドディレクトリの中に置く (方式ごとの設定が混ざらないように)
    $common = @("-B", $bdir, "-D", "SDKCONFIG=$bdir/sdkconfig", "-D", "SDKCONFIG_DEFAULTS=$defaults")
    if (-not (Test-Path "$bdir/sdkconfig")) {
        idf.py @common set-target $Target
        if ($LASTEXITCODE -ne 0) { throw "set-target に失敗しました" }
    }
    idf.py @common build
    if ($LASTEXITCODE -ne 0) { throw "ビルドに失敗しました" }
    idf.py @common size
    if ($Flash -ne "") {
        idf.py @common -p $Flash flash monitor
    }
} finally {
    Pop-Location
}
