#!/usr/bin/env python3
"""ESP32-DevKitC-VE の最初の USB 書き込みを1台ずつ続けて行い、MAC とノード番号の対応を CSV に残す (2026-11-15、GitHub issue #1)。

一度ビルドした同じ像 (node-esp32\\build.ps1 -Board devkitc-ve の出力) を、基板ごとに作り直さずに書く。
新しくつながった COM ポートを自動で見つけ、次の順に進める:
  1. esptool flash_id: MAC・チップ・実際のフラッシュの大きさ (Detected flash size) を読む。
     ビルドの想定 (flasher_args.json の flash_size) と違えば書かずに止める (4MB なら -Flash4MB で作り直す)。
  2. esptool write_flash: flasher_args.json のとおりに、ブートローダ・パーティションテーブル・otadata・アプリを書く。
  3. シリアルを開き、基板をリセットして起動のログを最長 --boot-seconds 秒読む。
  4. 起動のログを確かめる: PSRAM (Found 8MB PSRAM device と SPI SRAM memory test OK)、
     wire v2 envelope selftest: PASS、計算部品を積んだ後の内部 RAM の連続の空き (heap(after-wasm) の largest)、
     ノード番号 (起動の見出し "node id N")、Wi-Fi の MAC、書いた像と動いている像の一致 (ELF の SHA-256 の先頭)、
     異常終了・再起動の繰り返しが無いこと。
  5. CSV に1行追記し、ノード番号を大きく表示する (基板のラベル書き用)。
  6. 基板が外されるのを待ち、次の基板を待つ。Ctrl+C で終わる。

ノード番号の決まり方 (node-esp32\\main\\main.c の dyne_node_id): menuconfig の CONFIG_DYNE_NODE_ID が -1 (既定) なら
Wi-Fi の MAC (ESP_MAC_WIFI_STA。ESP32 では esptool が表示する MAC と同じ) の末尾1バイト (0〜255)。
このツールは起動のログの番号を正とし、esptool の MAC から計算した番号と食い違えば不合格にする。
末尾1バイトなので、別の基板と番号が重なることがある (15台なら3割ほどの確率)。CSV に同じ番号で別の MAC が
あれば不合格 (dup_id) にして知らせる。

出力 (既定は first_flash_out\\。git 管理外): first_flash.csv (1台1行) と logs\\<時刻>_<MAC>.log (esptool と起動のログ全文)。
CSV の列: timestamp, port, usb_serial, mac, node_id, id_from_mac, chip, flash_detected, psram_mb, psram_memtest,
  selftest, free_after_wasm, largest_after_wasm, fw_version, fw_elf_sha, result (PASS / FAIL), problems (不合格の理由の符号), log_file

必要なもの: pyserial (リポジトリの .venv にある)。esptool は ESP-IDF の Python 環境のもの
(C:\\Espressif\\python_env\\idf*_env) を自動で探す。--esptool で明示もできる。ESP-IDF の初期化は要らない。

使い方 (リポジトリのルートで):
  先にビルドだけしておく (書き込みはしない):
    powershell -ExecutionPolicy Bypass -File node-esp32\\build.ps1 -Board devkitc-ve
  何台も続けて書く (基板を1台ずつ USB につなぎ、表示を見て外し、次をつなぐ):
    .venv\\Scripts\\python.exe scripts\\first_flash.py
    (または powershell -ExecutionPolicy Bypass -File scripts\\first_flash.ps1)
  つないだままの1台だけ:
    .venv\\Scripts\\python.exe scripts\\first_flash.py --port COM5
  書かずに確かめて記録だけ (既に書いた基板の MAC と番号を記録し直す):
    .venv\\Scripts\\python.exe scripts\\first_flash.py --skip-flash --port COM5
  保存したログを確かめ直す (ハードウェア不要):
    .venv\\Scripts\\python.exe scripts\\first_flash.py --parse-log first_flash_out\\logs\\XXXX.log
  主なオプション:
    --build-dir DIR     既定 node-esp32\\build-ota-devkitc-ve (4MB 版なら node-esp32\\build-ota-devkitc-ve-4mb)
    --psram-mb N        期待する PSRAM の大きさ (既定 8。PSRAM の無い基板は 0 = 確かめない)
    --min-largest B     after-wasm の largest がこれ未満なら不合格 (既定 0 = 記録だけ)
    --boot-seconds S    起動のログを読む最長の秒数 (既定 15)
    --any-port          USB シリアルの変換 IC を CP2102N (VID 10C4 / PID EA60) に限らない
    --include-present   起動時に既につながっているポートも対象にする
    --erase             書く前にフラッシュ全体を消す (使い回しの基板。NVS・計算部品の保存区画も消える)
    --out DIR           出力の置き場所 (既定 first_flash_out)

書き込みが始まらない (Connecting....___ が続く) ときは、BOOT ボタンを押したまま待つ。ツールは数回やり直す。
説明: docs/2026-11-13-firmware-nodes.md §9.0
"""
import argparse
import csv
import datetime
import glob
import json
import os
import re
import shlex
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_BUILD = os.path.join(REPO, "node-esp32", "build-ota-devkitc-ve")
DEFAULT_OUT = os.path.join(REPO, "first_flash_out")
CP2102N_VID_PID = (0x10C4, 0xEA60)

CSV_FIELDS = ["timestamp", "port", "usb_serial", "mac", "node_id", "id_from_mac", "chip", "flash_detected",
              "psram_mb", "psram_memtest", "selftest", "free_after_wasm", "largest_after_wasm", "fw_version",
              "fw_elf_sha", "result", "problems", "log_file"]

# 不合格の理由の符号 → 説明 (CSV には符号だけを書く)
PROBLEM_TEXT = {
    "connect": "esptool が基板とつながらなかった (BOOT ボタンを押したまま、もう一度つなぐ)",
    "no_mac": "esptool の出力に MAC が無い",
    "chip": "チップが想定と違う",
    "flash_size": "フラッシュの大きさがビルドの想定と違う (4MB なら build.ps1 -Flash4MB で作り直し、--build-dir で指定)",
    "write": "書き込みが最後まで終わらなかった (Hash of data verified. が足りない)",
    "download_mode": "書き込みの状態のまま起動した (BOOT ボタンを離して EN ボタンを押す)",
    "no_boot_log": "起動のログが読めなかった",
    "psram": "PSRAM が見つからない、または大きさが想定と違う",
    "psram_memtest": "PSRAM の読み書きの検査 (SPI SRAM memory test) に失敗した",
    "selftest": "wire v2 envelope selftest が PASS でない",
    "wasm_fail": "計算部品を積めなかった (heap(wasm-fail))",
    "no_wasm_line": "heap(after-wasm) の行が無い (起動が途中で止まった?)",
    "largest_small": "after-wasm の largest が --min-largest より小さい",
    "no_node_id": "起動の見出しにノード番号が無い",
    "id_mismatch": "起動のログのノード番号が、MAC から計算した番号 (または固定の番号) と違う",
    "wifi_mac_mismatch": "Wi-Fi の MAC が esptool の MAC と違う",
    "fw_mismatch": "動いている像の ELF の SHA-256 が、書いた像と違う",
    "panic": "異常終了 (Guru Meditation・abort・ブラウンアウトなど)",
    "reboot": "起動を繰り返した",
    "dup_id": "同じノード番号の別の基板が CSV に既にある (番号の重なり。運用前に解消が要る)",
}

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
RST_RE = re.compile(r"^rst:0x([0-9a-fA-F]+) \(([A-Za-z0-9_]+)\),boot:0x([0-9a-fA-F]+)(?: \(([^)]*)\))?", re.M)
PANIC_RE = re.compile(r"(Guru Meditation Error[^\n]*|abort\(\) was called[^\n]*|Brownout detector was triggered"
                      r"|assert failed[^\n]*|Stack smashing protect failure|ESP_ERROR_CHECK failed[^\n]*)")
MAC_RE = r"((?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2})"


# ==== 文字列の解釈 (純粋な関数。tests: scripts/tests/test_first_flash.py) ====

def normalize_mac(s):
    return s.strip().lower() if s else None


def node_id_from_mac(mac):
    """node-esp32 の dyne_node_id() と同じ: Wi-Fi の MAC の末尾1バイト (CONFIG_DYNE_NODE_ID = -1 のとき)"""
    return int(mac.split(":")[-1], 16)


def parse_esptool(text):
    """esptool (4.x) の出力から MAC・チップ・フラッシュの大きさ・書き込みの検証の回数を読む"""
    text = ANSI_RE.sub("", text.replace("\r", "\n"))
    info = {"connected": False, "chip": None, "mac": None, "flash_detected": None, "hash_verified": 0,
            "fatal": None}
    m = re.search(r"^Chip is (.+?)\s*$", text, re.M)
    if m:
        info["connected"] = True
        info["chip"] = m.group(1)
    # EUI64 を持つチップは "MAC:" が 8 バイトで "BASE MAC:" が別にある。ESP32 は "MAC:" だけ
    m = re.search(r"^BASE MAC:\s*" + MAC_RE, text, re.M) or re.search(r"^MAC:\s*" + MAC_RE + r"\s*$", text, re.M)
    if m:
        info["mac"] = normalize_mac(m.group(1))
    sizes = re.findall(r"(?:Detected flash size|Auto-detected Flash size):\s*(\S+)", text)
    if sizes:
        info["flash_detected"] = sizes[-1]
    info["hash_verified"] = len(re.findall(r"Hash of data verified\.", text))
    m = re.search(r"A fatal error occurred:\s*(.+)", text)
    if m:
        info["fatal"] = m.group(1).strip()
    return info


def _split_boots(text):
    """ROM の "rst:0x..,boot:0x.." の行で起動ごとに分ける。返り値: (最初の rst より前, [(rst の一致, 本文), ...])"""
    ms = list(RST_RE.finditer(text))
    if not ms:
        return text, []
    boots = []
    for i, m in enumerate(ms):
        end = ms[i + 1].start() if i + 1 < len(ms) else len(text)
        boots.append((m, text[m.end():end]))
    return text[:ms[0].start()], boots


def parse_boot_log(text):
    """起動のログ (シリアル 115200) を解釈する。値は最初の起動から読み、異常終了は全体から探す"""
    text = ANSI_RE.sub("", text.replace("\r\n", "\n").replace("\r", "\n"))
    pre, boots = _split_boots(text)
    first = boots[0][1] if boots else text
    after_first_rst = text[boots[0][0].start():] if boots else text
    r = {
        "boots": len(boots),
        "reset_reasons": [m.group(2) for m, _ in boots],
        "download_mode": False,
        "node_id": None,
        "psram_mb": None,
        "psram_memtest": None,
        "psram_error": None,
        "selftest": None,
        "heap": {},
        "wasm": None,
        "wifi_mac": None,
        "elf_sha": None,
        "app_version": None,
        "panic": None,
    }
    for m, _ in boots:
        if "DOWNLOAD" in (m.group(4) or "").upper():
            r["download_mode"] = True
    if "waiting for download" in after_first_rst:
        r["download_mode"] = True
    m = re.search(r"==== Dyne node-esp32 \(node id (-?\d+)\) ====", first)
    if m:
        r["node_id"] = int(m.group(1))
    # ESP-IDF 5.x は "Found 8MB PSRAM device"、4.x は "Found 8MB SPI RAM device"
    m = re.search(r"Found (\d+)MB (?:PSRAM device|SPI RAM device)", first)
    if m:
        r["psram_mb"] = int(m.group(1))
    if re.search(r"SPI SRAM memory test OK", first):
        r["psram_memtest"] = "ok"
    elif re.search(r"SPI SRAM memory test fail", first):
        r["psram_memtest"] = "fail"
    m = re.search(r"(PSRAM ID read error[^\n]*|PSRAM enabled but initialization failed[^\n]*"
                  r"|PSRAM chip not found[^\n]*|Failed to init external RAM[^\n]*)", after_first_rst)
    if m:
        r["psram_error"] = m.group(1).strip()
    m = re.search(r"\[dyne\] wire v2 envelope selftest: (\w+)", first)
    if m:
        r["selftest"] = m.group(1)
    for m in re.finditer(r"\[dyne\] heap\(([\w-]+)\): free=(\d+) largest=(\d+)", first):
        r["heap"].setdefault(m.group(1), (int(m.group(2)), int(m.group(3))))
    if "after-wasm" in r["heap"]:
        r["wasm"] = "ok"
    elif "wasm-fail" in r["heap"]:
        r["wasm"] = "fail"
    m = re.search(r"wifi:mode\s*:\s*sta\s*\(" + MAC_RE + r"\)", first)
    if m:
        r["wifi_mac"] = normalize_mac(m.group(1))
    m = re.search(r"ELF file SHA256:\s*([0-9a-fA-F]+)", first)
    if m:
        r["elf_sha"] = m.group(1).lower()
    m = re.search(r"App version:\s*(\S+)", first)
    if m:
        r["app_version"] = m.group(1)
    m = PANIC_RE.search(after_first_rst)
    if m:
        r["panic"] = m.group(1).strip()
    return r


def boot_capture_done(text):
    """起動のログを読み終えてよいか: 計算部品の後のヒープの行と Wi-Fi の MAC の行がそろった、または異常終了"""
    t = ANSI_RE.sub("", text)
    if PANIC_RE.search(t) or "waiting for download" in t:
        return True
    return bool(re.search(r"\[dyne\] heap\((after-wasm|wasm-fail)\)", t)) and bool(re.search(r"wifi:mode\s*:\s*sta", t))


def evaluate(esp, boot, expect):
    """合否を決める。esp = parse_esptool の結果、boot = parse_boot_log の結果 (None = 読めなかった)。
    expect: flash_size ("8MB" など)、chip_prefix ("ESP32")、psram_mb (0 = 確かめない)、min_largest、
            n_files (書いたファイルの数。0 = 書いていない)、fixed_id (None = MAC から)、elf_sha (書いた像。None = 照合しない)
    返り値: (result "PASS"/"FAIL", 不合格の符号のリスト, 注意の文のリスト)"""
    probs, notes = [], []
    if not esp.get("connected"):
        probs.append("connect")
        return "FAIL", probs, notes
    if not esp.get("mac") and expect.get("need_mac", True):
        probs.append("no_mac")
    chip_prefix = expect.get("chip_prefix")
    if chip_prefix and esp.get("chip") and not esp["chip"].upper().startswith(chip_prefix.upper()):
        probs.append("chip")
    if expect.get("flash_size") and esp.get("flash_detected") != expect["flash_size"]:
        probs.append("flash_size")
    if expect.get("n_files", 0) > 0 and esp.get("hash_verified", 0) < expect["n_files"]:
        probs.append("write")
    if boot is None:
        probs.append("no_boot_log")
        return "FAIL", probs, notes
    if boot["download_mode"]:
        probs.append("download_mode")
    if expect.get("psram_mb", 0) > 0:
        if boot["psram_error"] or boot["psram_mb"] != expect["psram_mb"]:
            probs.append("psram")
        if boot["psram_memtest"] == "fail":
            probs.append("psram_memtest")
        elif boot["psram_memtest"] is None and boot["psram_mb"] is not None:
            notes.append("PSRAM の読み書きの検査の行 (SPI SRAM memory test OK) が無い (CONFIG_SPIRAM_MEMTEST が無効?)")
    if boot["selftest"] != "PASS":
        probs.append("selftest")
    if boot["wasm"] == "fail":
        probs.append("wasm_fail")
    elif boot["wasm"] is None:
        probs.append("no_wasm_line")
    elif expect.get("min_largest", 0) > 0 and boot["heap"]["after-wasm"][1] < expect["min_largest"]:
        probs.append("largest_small")
    if boot["node_id"] is None:
        probs.append("no_node_id")
    else:
        want = expect.get("fixed_id")
        mac = esp.get("mac") or boot["wifi_mac"]
        if want is None and mac:
            want = node_id_from_mac(mac)
        if want is not None and boot["node_id"] != want:
            probs.append("id_mismatch")
    if boot["wifi_mac"] is None:
        notes.append("Wi-Fi の MAC の行 (wifi:mode : sta) が読めなかった (MAC は esptool の値を使う)")
    elif esp.get("mac") and boot["wifi_mac"] != esp["mac"]:
        probs.append("wifi_mac_mismatch")
    if expect.get("elf_sha"):
        if boot["elf_sha"] is None:
            notes.append("ELF の SHA-256 の行が無く、書いた像との照合ができなかった")
        elif not expect["elf_sha"].lower().startswith(boot["elf_sha"]):
            probs.append("fw_mismatch")
    if boot["panic"]:
        probs.append("panic")
    if boot["boots"] > 1:
        probs.append("reboot")
    elif boot["boots"] == 0:
        notes.append("ROM の起動の行 (rst:0x...) が無い (リセットの前からのログの途中だけかもしれない)")
    return ("PASS" if not probs else "FAIL"), probs, notes


def find_duplicate_ids(rows, mac, node_id):
    """CSV の既存の行のうち、同じノード番号で MAC が違うものの MAC の一覧 (同じ基板の書き直しは数えない)"""
    if node_id is None or not mac:
        return []
    out = []
    for row in rows:
        if str(row.get("node_id", "")) == str(node_id) and row.get("mac") and row["mac"].lower() != mac.lower():
            if row["mac"].lower() not in out:
                out.append(row["mac"].lower())
    return out


def flash_plan(flasher_args, build_dir):
    """flasher_args.json (ESP-IDF のビルドの出力) から esptool の引数を作る。
    返り値: dict(chip, app (アプリの像のパス), flash_size, before, after, stub,
                 write_args = [--flash_mode .. 番地 ファイル ..], files = [(番地, パス)])"""
    files = sorted(((int(a, 16), os.path.normpath(os.path.join(build_dir, f)))
                    for a, f in flasher_args["flash_files"].items()), key=lambda x: x[0])
    extra = flasher_args.get("extra_esptool_args", {})
    write_args = list(flasher_args.get("write_flash_args", []))
    for a, f in files:
        write_args += [hex(a), f]
    app = flasher_args.get("app", {}).get("file")
    return {"chip": extra.get("chip", "esp32"), "app": os.path.normpath(os.path.join(build_dir, app)) if app else None, "flash_size": flasher_args.get("flash_settings", {}).get("flash_size"),
            "before": extra.get("before", "default_reset"), "after": extra.get("after", "hard_reset"),
            "stub": extra.get("stub", True), "write_args": write_args, "files": files}


def read_fixed_node_id(sdkconfig_text):
    """sdkconfig の CONFIG_DYNE_NODE_ID。-1 (MAC から) または見つからなければ None"""
    m = re.search(r"^CONFIG_DYNE_NODE_ID=(-?\d+)", sdkconfig_text, re.M)
    if not m or int(m.group(1)) < 0:
        return None
    return int(m.group(1))


def pick_new_port(known, ports, vid_pid=CP2102N_VID_PID):
    """ports = [(device, vid, pid), ...] (今つながっているもの)。known に無く、vid_pid に合う最初のポート名。
    vid_pid = None なら変換 IC で絞らない"""
    for dev, vid, pid in sorted(ports):
        if dev in known:
            continue
        if vid_pid is not None and (vid, pid) != vid_pid:
            continue
        return dev
    return None


def split_saved_log(text):
    """このツールが保存したログを (esptool の部分, 起動のログの部分) に分ける。印が無ければ全体を起動のログとみなす"""
    a, b = "=== esptool ===\n", "=== boot log ===\n"
    if a in text and b in text:
        i, j = text.index(a), text.index(b)
        return text[i + len(a):j], text[j + len(b):]
    return "", text


BIG_FONT = {
    "0": [" ### ", "#   #", "#   #", "#   #", " ### "],
    "1": ["  #  ", " ##  ", "  #  ", "  #  ", " ### "],
    "2": [" ### ", "#   #", "  ## ", " #   ", "#####"],
    "3": ["#### ", "    #", " ### ", "    #", "#### "],
    "4": ["#  # ", "#  # ", "#####", "   # ", "   # "],
    "5": ["#####", "#    ", "#### ", "    #", "#### "],
    "6": [" ### ", "#    ", "#### ", "#   #", " ### "],
    "7": ["#####", "    #", "   # ", "  #  ", "  #  "],
    "8": [" ### ", "#   #", " ### ", "#   #", " ### "],
    "9": [" ### ", "#   #", " ####", "    #", " ### "],
    "?": [" ### ", "#   #", "  ## ", "     ", "  #  "],
}


def big_text(s):
    """数字を 5 行の大きな文字にする (ラベル書きのため)"""
    rows = ["", "", "", "", ""]
    for ch in str(s):
        g = BIG_FONT.get(ch, BIG_FONT["?"])
        for i in range(5):
            rows[i] += g[i] + "  "
    return "\n".join(r.rstrip() for r in rows)


# ==== ハードウェアとのやりとり ====

def find_esptool():
    """esptool の起動の仕方: この Python に esptool があればそれ、無ければ ESP-IDF の Python 環境"""
    try:
        import esptool  # noqa: F401
        return [sys.executable, "-m", "esptool"]
    except ImportError:
        pass
    cands = sorted(glob.glob(r"C:\Espressif\python_env\idf*_env\Scripts\python.exe"))
    if cands:
        return [cands[-1], "-m", "esptool"]
    raise SystemExit("esptool が見つからない。--esptool で指定する (例: --esptool \"C:\\Espressif\\python_env\\"
                     "idf5.5_py3.11_env\\Scripts\\python.exe -m esptool\")")


def run_esptool(cmd, echo=True):
    """esptool を実行し、出力を表示しながら集める。返り値: (終了コード, 出力)"""
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1")
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
                         text=True, encoding="utf-8", errors="replace", bufsize=1)
    out = []
    for line in p.stdout:
        out.append(line)
        if echo:
            s = line.rstrip("\n")
            if s.startswith("Writing at"):
                print("  " + s, end="\r", flush=True)
            else:
                print("  " + s, flush=True)
    p.wait()
    return p.returncode, "".join(out)


def list_ports():
    from serial.tools import list_ports
    return [(p.device, p.vid, p.pid, p.serial_number) for p in list_ports.comports()]


def wait_new_port(known, vid_pid):
    """新しいポートがつながるまで待つ。known は今つながっているポートの集合 (外れたものは消していく)"""
    while True:
        now = list_ports()
        devs = {d for d, _, _, _ in now}
        known &= devs  # 外れたポートは忘れる (同じ基板をつなぎ直しても見つけられる)
        dev = pick_new_port(known, [(d, v, p) for d, v, p, _ in now], vid_pid)
        if dev:
            known.add(dev)
            sn = next((s for d, _, _, s in now if d == dev), None)
            return dev, sn
        time.sleep(0.5)


def wait_port_gone(port):
    while any(d == port for d, _, _, _ in list_ports()):
        time.sleep(0.5)


def capture_boot(port, seconds, baud=115200, echo=False):
    """シリアルを開いて基板をリセットし (RTS = EN を下げる。DTR = IO0 は上げたまま)、起動のログを読む"""
    import serial
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, 0.1
    ser.dtr, ser.rts = False, False
    ser.open()
    buf = bytearray()
    try:
        ser.reset_input_buffer()
        ser.dtr = False
        ser.rts = True
        time.sleep(0.2)
        ser.rts = False
        ser.dtr = False  # Windows の usbser.sys で RTS の変化を確実に反映させるため (idf_monitor と同じ)
        t0, t_done = time.time(), None
        while time.time() - t0 < seconds:
            chunk = ser.read(4096)
            if chunk:
                buf += chunk
                if echo:
                    sys.stdout.write(chunk.decode("utf-8", errors="replace"))
                    sys.stdout.flush()
            if t_done is None and boot_capture_done(buf.decode("utf-8", errors="replace")):
                t_done = time.time()
            if t_done is not None and time.time() - t_done > 1.0:  # 異常終了の後の情報も少し読む
                break
    finally:
        ser.close()
    return buf.decode("utf-8", errors="replace")


# ==== 記録 ====

def read_rows(csv_path):
    if not os.path.exists(csv_path):
        return []
    with open(csv_path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def append_row(csv_path, row):
    new = not os.path.exists(csv_path)
    with open(csv_path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if new:
            w.writeheader()
        w.writerow(row)


def make_row(ts, port, usb_serial, esp, boot, result, probs, fw, log_file):
    b = boot or {}
    heap = b.get("heap", {}).get("after-wasm") or b.get("heap", {}).get("wasm-fail") or (None, None)
    mac = esp.get("mac") or b.get("wifi_mac")
    return {
        "timestamp": ts, "port": port or "", "usb_serial": usb_serial or "", "mac": mac or "",
        "node_id": "" if b.get("node_id") is None else b["node_id"],
        "id_from_mac": node_id_from_mac(mac) if mac else "",
        "chip": esp.get("chip") or "", "flash_detected": esp.get("flash_detected") or "",
        "psram_mb": "" if b.get("psram_mb") is None else b["psram_mb"],
        "psram_memtest": b.get("psram_memtest") or "", "selftest": b.get("selftest") or "",
        "free_after_wasm": "" if heap[0] is None else heap[0],
        "largest_after_wasm": "" if heap[1] is None else heap[1],
        "fw_version": b.get("app_version") or fw.get("version", ""),
        "fw_elf_sha": b.get("elf_sha") or (fw.get("elf_sha256") or "")[:8],
        "result": result, "problems": ";".join(probs), "log_file": log_file or "",
    }


def report(result, probs, notes, boot, esp, dups):
    print()
    for n in notes:
        print(f"  注意: {n}")
    for p in probs:
        extra = ""
        if p == "dup_id":
            extra = f" ({', '.join(dups)})"
        elif p == "flash_size":
            extra = f" (検出 {esp.get('flash_detected')})"
        elif p == "psram" and boot:
            extra = f" ({boot['psram_error'] or str(boot['psram_mb']) + 'MB'})"
        elif p == "panic" and boot:
            extra = f" ({boot['panic']})"
        elif p == "reboot" and boot:
            extra = f" (起動の理由の並び: {', '.join(boot['reset_reasons'])})"
        print(f"  不合格: [{p}] {PROBLEM_TEXT.get(p, p)}{extra}")
    if boot and "after-wasm" in boot["heap"]:
        f, lg = boot["heap"]["after-wasm"]
        print(f"  計算部品を積んだ後の内部 RAM: 空き {f} B、連続の最大 {lg} B")
    nid = boot.get("node_id") if boot else None
    print()
    print(f"==== {result}   MAC {esp.get('mac') or '?'}   ノード番号 {'?' if nid is None else nid} ====")
    if nid is not None:
        print(big_text(nid))
    print()


def process_board(args, plan, fw, esptool_cmd, port, usb_serial, csv_path, log_dir):
    ts = datetime.datetime.now().isoformat(timespec="seconds")
    base = esptool_cmd + ["--chip", plan["chip"], "-p", port, "-b", str(args.baud)]
    if not plan["stub"]:
        base.append("--no-stub")
    print(f"\n---- {port} (USB serial {usb_serial or '?'})  {ts} ----")
    esp_text = ""
    for attempt in range(args.retries + 1):
        rc, out = run_esptool(base + ["flash_id"])
        esp_text += out
        if rc == 0:
            break
        if attempt < args.retries:
            print("  つながらない。BOOT ボタンを押したまま待つ (やり直します)")
            time.sleep(2)
    esp = parse_esptool(esp_text)
    expect = {"flash_size": plan["flash_size"], "chip_prefix": plan["chip"].upper(), "psram_mb": args.psram_mb,
              "min_largest": args.min_largest, "n_files": 0 if args.skip_flash else len(plan["files"]),
              "fixed_id": args.fixed_id, "elf_sha": fw.get("elf_sha256")}
    boot = None
    boot_text = ""
    stop = (not esp["connected"]) or (esp.get("flash_detected") != plan["flash_size"] and not args.force)
    if stop:
        if esp["connected"]:
            print("  書かずに止めます (--force で、フラッシュの大きさが違っても書く)")
    else:
        if not args.skip_flash:
            if args.erase:
                rc, out = run_esptool(base + ["erase_flash"])
                esp_text += out
            rc, out = run_esptool(base + ["--before", plan["before"], "--after", plan["after"], "write_flash"]
                                  + plan["write_args"])
            esp_text += out
            esp = parse_esptool(esp_text)
        if args.skip_flash or rc == 0:
            print(f"  起動のログを読みます (最長 {args.boot_seconds} 秒)")
            time.sleep(0.5)
            try:
                boot_text = capture_boot(port, args.boot_seconds, echo=args.echo)
                boot = parse_boot_log(boot_text) if boot_text.strip() else None
            except Exception as e:  # ポートが開けない等
                print(f"  シリアルを開けなかった: {e}")
    result, probs, notes = evaluate(esp, boot, expect)
    rows = read_rows(csv_path)
    mac = esp.get("mac") or (boot or {}).get("wifi_mac")
    dups = find_duplicate_ids(rows, mac, boot.get("node_id") if boot else None)
    if dups:
        probs.append("dup_id")
        result = "FAIL"
    mac_tag = (mac or "nomac").replace(":", "")
    log_file = os.path.join(log_dir, f"{ts.replace(':', '').replace('-', '')}_{mac_tag}.log")
    with open(log_file, "w", encoding="utf-8", newline="\n") as f:
        f.write(f"# first_flash.py {ts} port={port} usb_serial={usb_serial or ''}\n")
        f.write("=== esptool ===\n" + esp_text.replace("\r\n", "\n").replace("\r", "\n"))
        f.write("\n=== boot log ===\n" + boot_text.replace("\r\n", "\n").replace("\r", "\n"))
    append_row(csv_path, make_row(ts, port, usb_serial, esp, boot, result, probs, fw,
                                  os.path.relpath(log_file, os.path.dirname(csv_path))))
    report(result, probs, notes, boot, esp, dups)
    return result


def load_firmware_info(app):
    """書く像の版と ELF の SHA-256 (dyne_ota.py の像の解釈を使う。読めなければ空)"""
    if not app or not os.path.exists(app):
        return {}
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import dyne_ota
        with open(app, "rb") as f:
            b = f.read()
        ok, why = dyne_ota.verify_image(b)
        if not ok:
            raise SystemExit(f"アプリの像 {app} が壊れている: {why}")
        return dyne_ota.parse_image(b).get("desc", {})
    except (ImportError, ValueError):
        return {}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--build-dir", default=DEFAULT_BUILD)
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--port", help="このポートの1台だけを扱って終わる")
    ap.add_argument("--psram-mb", type=int, default=8)
    ap.add_argument("--min-largest", type=int, default=0)
    ap.add_argument("--boot-seconds", type=float, default=15)
    ap.add_argument("--baud", type=int, default=460800, help="書き込みの速さ (既定 460800)")
    ap.add_argument("--retries", type=int, default=3, help="esptool がつながらないときのやり直しの回数")
    ap.add_argument("--any-port", action="store_true")
    ap.add_argument("--include-present", action="store_true")
    ap.add_argument("--skip-flash", action="store_true")
    ap.add_argument("--erase", action="store_true")
    ap.add_argument("--force", action="store_true", help="フラッシュの大きさが想定と違っても書く")
    ap.add_argument("--echo", action="store_true", help="起動のログをそのまま表示する")
    ap.add_argument("--esptool", help='esptool の起動の仕方 (例: "C:\\...\\python.exe -m esptool")')
    ap.add_argument("--parse-log", help="保存したログ (または起動のログだけのテキスト) を確かめ直して終わる")
    args = ap.parse_args()

    build_dir = os.path.abspath(args.build_dir)
    fa_path = os.path.join(build_dir, "flasher_args.json")
    plan = None
    fw = {}
    args.fixed_id = None
    if os.path.exists(fa_path):
        with open(fa_path, encoding="utf-8") as f:
            plan = flash_plan(json.load(f), build_dir)
        sdk = os.path.join(build_dir, "sdkconfig")
        if os.path.exists(sdk):
            with open(sdk, encoding="utf-8", errors="replace") as f:
                args.fixed_id = read_fixed_node_id(f.read())
        fw = load_firmware_info(plan["app"])

    if args.parse_log:
        with open(args.parse_log, encoding="utf-8", errors="replace") as f:
            esp_text, boot_text = split_saved_log(f.read())
        esp = parse_esptool(esp_text) if esp_text else {"connected": True, "mac": None}
        boot = parse_boot_log(boot_text) if boot_text.strip() else None
        expect = {"flash_size": plan["flash_size"] if (plan and esp_text) else None, "chip_prefix": None,
                  "need_mac": bool(esp_text),
                  "psram_mb": args.psram_mb, "min_largest": args.min_largest, "n_files": 0,
                  "fixed_id": args.fixed_id, "elf_sha": None}
        result, probs, notes = evaluate(esp, boot, expect)
        report(result, probs, notes, boot, esp, [])
        return 0 if result == "PASS" else 1

    if plan is None:
        raise SystemExit(f"{fa_path} が無い。先に node-esp32\\build.ps1 -Board devkitc-ve でビルドする")
    for _, p in plan["files"]:
        if not os.path.exists(p):
            raise SystemExit(f"{p} が無い。ビルドをやり直す")
    if args.fixed_id is not None:
        print(f"警告: このビルドはノード番号を {args.fixed_id} に固定している (CONFIG_DYNE_NODE_ID)。"
              "全台が同じ番号になる。-1 (MAC から) でビルドし直すことを勧める")
    esptool_cmd = [t.strip('"') for t in shlex.split(args.esptool, posix=False)] if args.esptool else find_esptool()
    os.makedirs(args.out, exist_ok=True)
    log_dir = os.path.join(args.out, "logs")
    os.makedirs(log_dir, exist_ok=True)
    csv_path = os.path.join(args.out, "first_flash.csv")
    print(f"像: {build_dir} (チップ {plan['chip']}、フラッシュ {plan['flash_size']}、版 {fw.get('version', '?')}、"
          f"ELF SHA-256 {(fw.get('elf_sha256') or '?')[:8]})")
    print(f"esptool: {' '.join(esptool_cmd)}")
    print(f"記録: {csv_path} (git 管理外)")

    if args.port:
        sn = next((s for d, _, _, s in list_ports() if d == args.port), None)
        r = process_board(args, plan, fw, esptool_cmd, args.port, sn, csv_path, log_dir)
        return 0 if r == "PASS" else 1

    vid_pid = None if args.any_port else CP2102N_VID_PID
    known = set() if args.include_present else {d for d, _, _, _ in list_ports()}
    n_ok = n_ng = 0
    try:
        while True:
            print(f"\n次の基板を USB でつないでください (済み: 合格 {n_ok} 台・不合格 {n_ng} 台。終わりは Ctrl+C)")
            port, sn = wait_new_port(known, vid_pid)
            time.sleep(1.0)  # 認識の直後はポートが開けないことがある
            r = process_board(args, plan, fw, esptool_cmd, port, sn, csv_path, log_dir)
            if r == "PASS":
                n_ok += 1
            else:
                n_ng += 1
            print(f"ラベルを書いたら {port} の基板を外してください")
            wait_port_gone(port)
    except KeyboardInterrupt:
        print(f"\n終わり: 合格 {n_ok} 台・不合格 {n_ng} 台。記録: {csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
