# node-esp32 の OTA 対応版のビルド (2026-11-11、基板の切り替えを 2026-11-13 に追加)。基板ごとに別のビルドディレクトリを使う。
#   使い方: powershell -ExecutionPolicy Bypass -File node-esp32\build.ps1 [-Board espr|devkitc-ve|xiao-c5] [-Flash4MB] [-Flash COM5] [-Menuconfig] [-Reconfigure]
#   例:     powershell -ExecutionPolicy Bypass -File node-esp32\build.ps1 -Board devkitc-ve
#   -Board espr       : ESPr Developer 32 (ESP32-WROOM-32E、4MB、PSRAM なし)。ビルドディレクトリ build-ota-esp32/ (従来と同じ)
#   -Board devkitc-ve : ESP32-DevKitC-VE (ESP32-WROVER-E、PSRAM 8MB、フラッシュ 8MB)。build-ota-devkitc-ve/
#   -Board xiao-c5    : XIAO ESP32C5 (8MB)。build-ota-esp32c5/ (従来と同じ)
#   -Target esp32|esp32c5 : 従来の指定 (esp32 = espr、esp32c5 = xiao-c5 と同じ)。-Board があればそちらが優先
#   -Flash4MB   : フラッシュ 4MB の基板 (区画表 partitions_4mb.csv)。DevKitC-VE が 4MB だった場合に使う
#   -Menuconfig : ビルドの前に menuconfig を開く。ただし Wi-Fi の SSID・パスワード・ブローカーの URI は
#                 node-esp32\sdkconfig.local (git 管理外) に書くのを勧める (下の自動の作り直しで消えないため)。例:
#                   CONFIG_DYNE_WIFI_SSID="dyne-exp1"
#                   CONFIG_DYNE_WIFI_PASS="..."
#                   CONFIG_DYNE_BROKER_URI="mqtt://192.168.137.1:1883"
#   -Reconfigure: ビルドディレクトリの sdkconfig を消して、既定の設定から作り直す
#   -Flash COM5 : ビルド後に USB で書き込んでモニタを開く (実機作業。最初の1回だけ。以後は OTA)
# 既定の設定のファイル (sdkconfig.defaults・sdkconfig.board.<基板>・sdkconfig.local など) がビルドディレクトリの
# sdkconfig より新しければ、sdkconfig を作り直す (ESP-IDF は既にある sdkconfig の値を既定の設定で上書きしないため)。
# 説明: docs/2026-11-11-timesync-ota.md、docs/2026-11-13-firmware-nodes.md
param(
    [ValidateSet("", "espr", "devkitc-ve", "xiao-c5")] [string]$Board = "",
    [ValidateSet("esp32c5", "esp32")] [string]$Target = "esp32c5",
    [switch]$Flash4MB,
    [string]$Flash = "",
    [switch]$Menuconfig,
    [switch]$Reconfigure
)
$ErrorActionPreference = "Stop"
$here = $PSScriptRoot
. C:\Espressif\Initialize-Idf.ps1 -IdfId esp-idf-7a8737a10725bc5cb19d516a1186b39f

if ($Board -eq "") {
    $Board = if ($Target -eq "esp32") { "espr" } else { "xiao-c5" }
}
switch ($Board) {
    "espr"       { $chip = "esp32";   $bdir = "build-ota-esp32" }
    "devkitc-ve" { $chip = "esp32";   $bdir = "build-ota-devkitc-ve" }
    "xiao-c5"    { $chip = "esp32c5"; $bdir = "build-ota-esp32c5" }
}
# 重ねる順: 共通 → 基板 → (4MB) → 手元の設定。sdkconfig.defaults.<チップ> (あれば) は ESP-IDF が
# sdkconfig.defaults の直後に自動で重ねる
$defs = @("sdkconfig.defaults")
if (Test-Path "$here\sdkconfig.board.$Board") { $defs += "sdkconfig.board.$Board" }
if ($Flash4MB) { $defs += "sdkconfig.flash4mb"; $bdir = "$bdir-4mb" }
if (Test-Path "$here\sdkconfig.local") { $defs += "sdkconfig.local" }
$defList = $defs -join ";"

Push-Location $here
try {
    $sdk = "$bdir/sdkconfig"
    if (Test-Path $sdk) {
        $sdkTime = (Get-Item $sdk).LastWriteTime
        $all = @($defs) + @("sdkconfig.defaults.$chip") | Where-Object { Test-Path $_ }
        $newer = $all | Where-Object { (Get-Item $_).LastWriteTime -gt $sdkTime }
        if ($Reconfigure -or $newer) {
            Write-Host "sdkconfig を既定の設定から作り直します ($sdk。理由: $(if ($Reconfigure) { '-Reconfigure' } else { ($newer -join ', ') + ' が新しい' }))"
            Remove-Item $sdk
        }
    }
    $common = @("-B", $bdir, "-D", "SDKCONFIG=$sdk", "-D", "SDKCONFIG_DEFAULTS=$defList")
    if (-not (Test-Path $sdk)) {
        idf.py @common set-target $chip
        if ($LASTEXITCODE -ne 0) { throw "set-target に失敗しました" }
    }
    if ($Menuconfig) {
        idf.py @common menuconfig
    }
    idf.py @common build
    if ($LASTEXITCODE -ne 0) { throw "ビルドに失敗しました" }
    idf.py @common size
    if ($Flash -ne "") {
        # 最初の書き込みは区画表・ブートローダ・otadata の初期値を含む全体を書く
        idf.py @common -p $Flash flash monitor
    }
} finally {
    Pop-Location
}
