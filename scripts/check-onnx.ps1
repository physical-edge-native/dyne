# ONNX → .dynm → WASMアクター (crates/mlp-actor, crates/mlp-train-actor) の経路を一括で確認する
#   1. mlp-core (推論の部品) / mlp-actor / mlp-train-actor の単体テスト (ネイティブ)
#   2. WASM のビルド (フル版 / 実機向け縮小版 --features small)
#   3. scripts/onnx_check.py (onnxruntime・参照計算・WASMアクターの照合)
#   4. scripts/onnx_torch_check.py (PyTorch で学習した PINN の書き出し → WASMアクターとの照合、
#      学習用アクターでの追加学習)。torch が .venv に無ければ、その旨を表示して飛ばす
# 前提: リポジトリの .venv に numpy / onnx / onnxruntime / wasmtime が入っていること
#   .venv\Scripts\python.exe -m pip install numpy onnx onnxruntime wasmtime
#   (4 を行うなら) .venv\Scripts\python.exe -m pip install torch --index-url https://download.pytorch.org/whl/cpu
# 使い方: powershell -ExecutionPolicy Bypass -File scripts\check-onnx.ps1

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$py = Join-Path $root ".venv\Scripts\python.exe"

Push-Location (Join-Path $root "crates\mlp-core")
try {
    cargo test --release
    if ($LASTEXITCODE -ne 0) { throw "mlp-core の単体テストが失敗しました" }
} finally {
    Pop-Location
}

foreach ($name in "mlp-actor", "mlp-train-actor") {
    Push-Location (Join-Path $root "crates\$name")
    try {
        cargo test --release
        if ($LASTEXITCODE -ne 0) { throw "$name の単体テストが失敗しました" }
        cargo build --release --target wasm32-unknown-unknown
        if ($LASTEXITCODE -ne 0) { throw "$name の WASM (フル版) のビルドが失敗しました" }
        cargo build --release --target wasm32-unknown-unknown --features small --target-dir target\small
        if ($LASTEXITCODE -ne 0) { throw "$name の WASM (縮小版) のビルドが失敗しました" }
    } finally {
        Pop-Location
    }
}

$env:PYTHONIOENCODING = "utf-8"
& $py (Join-Path $root "scripts\onnx_check.py")
if ($LASTEXITCODE -ne 0) { throw "onnx_check.py が FAIL でした" }

# torch が無い場合は onnx_torch_check.py 自身が「飛ばします」と表示して終了コード 0 で抜ける
& $py -W ignore (Join-Path $root "scripts\onnx_torch_check.py")
if ($LASTEXITCODE -ne 0) { throw "onnx_torch_check.py が FAIL でした" }

# 5. 2次元の領域分割への組み込み: PyTorch で学習した (x, y) → u のモデルを各ノードに配り、
#    各ノードが自分の区画の初期場をそのモデルで作って熱拡散を解く (crates/heat2d-mlp-actor)。
#    分割解と単一ノード解のビット一致 (PASS (bit-exact)) を確かめる。torch が無ければ飛ばす
$work = Join-Path $env:TEMP "dyne-onnx-field2d"
New-Item -ItemType Directory -Force $work | Out-Null
$dynm = Join-Path $work "field2d.dynm"
& $py -W ignore (Join-Path $root "scripts\onnx_torch_field2d.py") -o $dynm
if ($LASTEXITCODE -eq 2) {
    Write-Host "torch が無いため、2次元への組み込みの確認は飛ばします"
} elseif ($LASTEXITCODE -ne 0) {
    throw "onnx_torch_field2d.py が失敗しました"
} else {
    Push-Location (Join-Path $root "crates\heat2d-mlp-actor")
    try {
        cargo test --release
        if ($LASTEXITCODE -ne 0) { throw "heat2d-mlp-actor の単体テストが失敗しました" }
        cargo build --release --target wasm32-unknown-unknown
        if ($LASTEXITCODE -ne 0) { throw "heat2d-mlp-actor の WASM のビルドが失敗しました" }
    } finally {
        Pop-Location
    }
    Push-Location $root
    try {
        cargo build --release -p dyne-sim2d
        if ($LASTEXITCODE -ne 0) { throw "dyne-sim2d のビルドが失敗しました" }
    } finally {
        Pop-Location
    }
    $exe = Join-Path $root "target\release\dyne-sim2d.exe"
    $wm = Join-Path $root "crates\heat2d-mlp-actor\target\wasm32-unknown-unknown\release\heat2d_mlp_actor.wasm"
    $cases = [ordered]@{
        "model_2x2"      = @("--px","2","--py","2","--steps","500")
        "model_3x2_loss" = @("--px","3","--py","2","--nx","40","--ny","33","--steps","400","--loss","0.15","--lat-ms","5","--lat-jitter-ms","10")
        "model_4x4_part" = @("--px","4","--py","4","--nx","61","--ny","61","--steps","300","--dt","4e-5","--loss","0.1","--pause","5@3000:8000","--partition","0-1@10000:15000")
        "model_zero_grad"= @("--px","2","--py","3","--steps","300","--outer-bc","zero_grad","--loss","0.1")
    }
    $fail = 0
    foreach ($k in $cases.Keys) {
        $lines = & $exe --wasm $wm --model $dynm --quiet @($cases[$k]) | Select-String -Pattern "PASS|FAIL|analytic"
        Write-Host ("{0,-16} {1}" -f $k, ($lines -join " | "))
        if (-not ($lines -match "PASS \(bit-exact\)")) { $fail++ }
    }
    if ($fail -ne 0) { throw "2次元への組み込みで FAIL が $fail 件ありました" }
}
