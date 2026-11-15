# scripts/first_flash.py (最初の USB 書き込みを続けて行う道具) の、ハードウェアの要らない部分のテスト (2026-11-15)。
# 実行 (リポジトリ直下で): .venv\Scripts\python.exe -m unittest scripts/tests/test_first_flash.py -v
#
# fixtures/first_flash/ のログはすべて合成したもの (実機の記録ではない)。文言は次から写した:
#   esptool 4.12.0 (ESP-IDF 5.5.5 の Python 環境) の cmds.py・__init__.py の print、
#   ESP-IDF 5.5.5 の components/esp_psram/system_layer/esp_psram.c・esp32/esp_psram_impl_quad.c・
#   esp_app_format/esp_app_desc.c のログ、node-esp32/main/main.c の printf。
# MAC は実機のものを使わず、02:00:00:00:00:xx (ローカル管理の番地) にしている。
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import first_flash as ff  # noqa: E402

FIX = os.path.join(HERE, "fixtures", "first_flash")


def fx(name):
    with open(os.path.join(FIX, name), encoding="utf-8") as f:
        return f.read()


EXPECT = {"flash_size": "8MB", "chip_prefix": "ESP32", "psram_mb": 8, "min_largest": 0, "n_files": 4,
          "fixed_id": None, "elf_sha": "1a2b3c4d" + "0" * 56}


class Esptool(unittest.TestCase):
    def test_flash_id_8mb(self):
        e = ff.parse_esptool(fx("esptool_flash_id_8mb.txt"))
        self.assertTrue(e["connected"])
        self.assertEqual(e["mac"], "02:00:00:00:00:2a")
        self.assertEqual(e["chip"], "ESP32-D0WD-V3 (revision v3.1)")
        self.assertEqual(e["flash_detected"], "8MB")
        self.assertEqual(e["hash_verified"], 0)

    def test_flash_id_4mb(self):
        self.assertEqual(ff.parse_esptool(fx("esptool_flash_id_4mb.txt"))["flash_detected"], "4MB")

    def test_no_connect(self):
        e = ff.parse_esptool(fx("esptool_no_connect.txt"))
        self.assertFalse(e["connected"])
        self.assertIsNone(e["mac"])
        self.assertIn("Failed to connect", e["fatal"])
        self.assertEqual(ff.evaluate(e, None, EXPECT), ("FAIL", ["connect"], []))

    def test_write(self):
        e = ff.parse_esptool(fx("esptool_write_ok.txt"))
        self.assertEqual(e["hash_verified"], 4)
        self.assertEqual(e["flash_detected"], "8MB")

    def test_carriage_returns(self):
        # 進み具合の表示は \r で上書きされる。\r だけの区切りでも読めること
        t = fx("esptool_flash_id_8mb.txt").replace("\n", "\r")
        self.assertEqual(ff.parse_esptool(t)["mac"], "02:00:00:00:00:2a")


class BootLog(unittest.TestCase):
    def test_ok(self):
        b = ff.parse_boot_log(fx("boot_ok.log"))
        self.assertEqual(b["boots"], 1)
        self.assertEqual(b["reset_reasons"], ["POWERON_RESET"])
        self.assertFalse(b["download_mode"])
        self.assertEqual(b["node_id"], 42)
        self.assertEqual(b["psram_mb"], 8)
        self.assertEqual(b["psram_memtest"], "ok")
        self.assertIsNone(b["psram_error"])
        self.assertEqual(b["selftest"], "PASS")
        self.assertEqual(b["heap"]["boot"], (262144, 114688))
        self.assertEqual(b["heap"]["after-wasm"], (180224, 98304))
        self.assertEqual(b["wasm"], "ok")
        self.assertEqual(b["wifi_mac"], "02:00:00:00:00:2a")
        self.assertEqual(b["elf_sha"], "1a2b3c4d")
        self.assertEqual(b["app_version"], "58cb7b4-dirty")
        self.assertIsNone(b["panic"])

    def test_ok_evaluate(self):
        e = ff.parse_esptool(fx("esptool_write_ok.txt"))
        b = ff.parse_boot_log(fx("boot_ok.log"))
        self.assertEqual(ff.evaluate(e, b, EXPECT), ("PASS", [], []))

    def test_crlf_and_colors(self):
        # シリアルの行末は \r\n。ログの色 (ANSI の制御文字) が有効なビルドでも読めること
        t = fx("boot_ok.log").replace("\n", "\r\n").replace("I (137) esp_psram", "\x1b[0;32mI (137) esp_psram")
        b = ff.parse_boot_log(t)
        self.assertEqual(b["psram_mb"], 8)
        self.assertEqual(b["node_id"], 42)

    def test_old_idf_wording(self):
        # docs §9.0 の文言 (ESP-IDF 4.x の "Found 8MB SPI RAM device") も読む
        t = fx("boot_ok.log").replace("Found 8MB PSRAM device", "Found 8MB SPI RAM device")
        self.assertEqual(ff.parse_boot_log(t)["psram_mb"], 8)

    def test_psram_missing(self):
        e = ff.parse_esptool(fx("esptool_write_ok.txt"))
        b = ff.parse_boot_log(fx("boot_psram_missing.log"))
        self.assertIn("PSRAM ID read error", b["psram_error"])
        self.assertIn("abort() was called", b["panic"])
        self.assertEqual(b["boots"], 2)
        self.assertEqual(b["reset_reasons"], ["POWERON_RESET", "SW_CPU_RESET"])
        res, probs, _ = ff.evaluate(e, b, EXPECT)
        self.assertEqual(res, "FAIL")
        for p in ("psram", "selftest", "no_wasm_line", "no_node_id", "panic", "reboot"):
            self.assertIn(p, probs)

    def test_psram_4mb(self):
        e = ff.parse_esptool(fx("esptool_write_ok.txt").replace("00:2a", "00:31"))
        b = ff.parse_boot_log(fx("boot_psram_4mb.log"))
        self.assertEqual(b["psram_mb"], 4)
        self.assertEqual(ff.evaluate(e, b, EXPECT)[1], ["psram"])
        # PSRAM の無い基板向け (psram_mb = 0) なら確かめない
        self.assertEqual(ff.evaluate(e, b, dict(EXPECT, psram_mb=0))[0], "PASS")

    def test_selftest_fail(self):
        e = ff.parse_esptool(fx("esptool_write_ok.txt"))
        b = ff.parse_boot_log(fx("boot_selftest_fail.log"))
        self.assertEqual(ff.evaluate(e, b, EXPECT), ("FAIL", ["selftest"], []))

    def test_wasm_fail(self):
        e = ff.parse_esptool(fx("esptool_write_ok.txt"))
        b = ff.parse_boot_log(fx("boot_wasm_fail.log"))
        self.assertEqual(b["wasm"], "fail")
        self.assertEqual(b["heap"]["wasm-fail"], (150000, 60000))
        self.assertEqual(ff.evaluate(e, b, EXPECT)[1], ["wasm_fail"])

    def test_min_largest(self):
        e = ff.parse_esptool(fx("esptool_write_ok.txt"))
        b = ff.parse_boot_log(fx("boot_ok.log"))
        self.assertEqual(ff.evaluate(e, b, dict(EXPECT, min_largest=98304))[0], "PASS")
        self.assertEqual(ff.evaluate(e, b, dict(EXPECT, min_largest=98305))[1], ["largest_small"])

    def test_download_mode(self):
        b = ff.parse_boot_log(fx("boot_download_mode.log"))
        self.assertTrue(b["download_mode"])
        self.assertTrue(ff.boot_capture_done(fx("boot_download_mode.log")))
        self.assertIn("download_mode", ff.evaluate(ff.parse_esptool(fx("esptool_write_ok.txt")), b, EXPECT)[1])

    def test_flash_size_mismatch(self):
        e = ff.parse_esptool(fx("esptool_flash_id_4mb.txt"))
        res, probs, _ = ff.evaluate(e, None, dict(EXPECT, n_files=0))
        self.assertEqual((res, probs), ("FAIL", ["flash_size", "no_boot_log"]))

    def test_incomplete_write(self):
        e = ff.parse_esptool(fx("esptool_write_ok.txt"))
        e["hash_verified"] = 3
        self.assertIn("write", ff.evaluate(e, ff.parse_boot_log(fx("boot_ok.log")), EXPECT)[1])

    def test_id_mismatch_and_wifi_mac(self):
        # 別の基板の esptool の出力と組み合わせると、番号と Wi-Fi の MAC の両方が食い違う
        e = ff.parse_esptool(fx("esptool_flash_id_4mb.txt").replace("4MB", "8MB"))
        e["hash_verified"] = 4
        probs = ff.evaluate(e, ff.parse_boot_log(fx("boot_ok.log")), EXPECT)[1]
        self.assertEqual(probs, ["id_mismatch", "wifi_mac_mismatch"])

    def test_fixed_id(self):
        e = ff.parse_esptool(fx("esptool_write_ok.txt"))
        b = ff.parse_boot_log(fx("boot_ok.log"))
        self.assertEqual(ff.evaluate(e, b, dict(EXPECT, fixed_id=42))[0], "PASS")
        self.assertEqual(ff.evaluate(e, b, dict(EXPECT, fixed_id=7))[1], ["id_mismatch"])

    def test_fw_mismatch(self):
        e = ff.parse_esptool(fx("esptool_write_ok.txt"))
        b = ff.parse_boot_log(fx("boot_ok.log"))
        self.assertEqual(ff.evaluate(e, b, dict(EXPECT, elf_sha="ffff" + "0" * 60))[1], ["fw_mismatch"])
        res, probs, notes = ff.evaluate(e, b, dict(EXPECT, elf_sha=None))
        self.assertEqual((res, probs, notes), ("PASS", [], []))

    def test_capture_done(self):
        t = fx("boot_ok.log")
        self.assertTrue(ff.boot_capture_done(t))
        cut = t[:t.index("I (1500) wifi:")]
        self.assertFalse(ff.boot_capture_done(cut))
        self.assertTrue(ff.boot_capture_done(fx("boot_psram_missing.log")))

    def test_boot_log_without_rst(self):
        # リセットの行より後だけを読んだ場合: 全体を1回の起動とみなし、注意を出す
        t = fx("boot_ok.log")
        t = t[t.index("I (31) boot:"):]
        b = ff.parse_boot_log(t)
        self.assertEqual(b["boots"], 0)
        res, probs, notes = ff.evaluate(ff.parse_esptool(fx("esptool_write_ok.txt")), b, EXPECT)
        self.assertEqual(res, "PASS")
        self.assertEqual(len(notes), 1)


class Helpers(unittest.TestCase):
    def test_node_id_from_mac(self):
        self.assertEqual(ff.node_id_from_mac("02:00:00:00:00:2a"), 42)
        self.assertEqual(ff.node_id_from_mac("02:00:00:00:01:FF"), 255)

    def test_duplicates(self):
        rows = [{"mac": "02:00:00:00:00:2a", "node_id": "42"}, {"mac": "02:00:00:00:01:2a", "node_id": "42"},
                {"mac": "02:00:00:00:00:07", "node_id": "7"}]
        self.assertEqual(ff.find_duplicate_ids(rows, "02:00:00:00:00:2a", 42), ["02:00:00:00:01:2a"])
        self.assertEqual(ff.find_duplicate_ids(rows, "02:00:00:00:00:08", 8), [])
        self.assertEqual(ff.find_duplicate_ids(rows, "02:00:00:00:00:07", 7), [])  # 同じ基板の書き直し

    def test_flash_plan(self):
        fa = {"write_flash_args": ["--flash_mode", "dio", "--flash_size", "8MB", "--flash_freq", "40m"],
              "flash_settings": {"flash_mode": "dio", "flash_size": "8MB", "flash_freq": "40m"},
              "flash_files": {"0x1000": "bootloader/bootloader.bin", "0x10000": "dyne_node.bin",
                              "0x8000": "partition_table/partition-table.bin", "0xd000": "ota_data_initial.bin"},
              "app": {"offset": "0x10000", "file": "dyne_node.bin", "encrypted": "false"},
              "extra_esptool_args": {"after": "hard_reset", "before": "default_reset", "stub": True, "chip": "esp32"}}
        p = ff.flash_plan(fa, os.path.join("B"))
        self.assertEqual(p["chip"], "esp32")
        self.assertEqual(p["flash_size"], "8MB")
        self.assertEqual([a for a, _ in p["files"]], [0x1000, 0x8000, 0xD000, 0x10000])
        self.assertEqual(p["app"], os.path.normpath("B/dyne_node.bin"))
        self.assertEqual(p["write_args"][:6], ["--flash_mode", "dio", "--flash_size", "8MB", "--flash_freq", "40m"])
        self.assertEqual(p["write_args"][6:8], ["0x1000", os.path.normpath("B/bootloader/bootloader.bin")])
        self.assertEqual(len(p["write_args"]), 6 + 8)

    def test_real_flasher_args_if_built(self):
        # ビルド済みなら実物の flasher_args.json も読めること (無ければ飛ばす)
        b = os.path.join(ff.DEFAULT_BUILD, "flasher_args.json")
        if not os.path.exists(b):
            self.skipTest("node-esp32/build-ota-devkitc-ve が無い")
        import json
        with open(b, encoding="utf-8") as f:
            p = ff.flash_plan(json.load(f), ff.DEFAULT_BUILD)
        self.assertEqual(p["chip"], "esp32")
        self.assertEqual(p["flash_size"], "8MB")
        for _, path in p["files"]:
            self.assertTrue(os.path.exists(path), path)

    def test_fixed_node_id(self):
        self.assertIsNone(ff.read_fixed_node_id("CONFIG_X=1\nCONFIG_DYNE_NODE_ID=-1\n"))
        self.assertEqual(ff.read_fixed_node_id("CONFIG_DYNE_NODE_ID=5\n"), 5)
        self.assertIsNone(ff.read_fixed_node_id(""))

    def test_pick_new_port(self):
        ports = [("COM3", 0x8087, 0x0AAA), ("COM7", 0x10C4, 0xEA60), ("COM9", 0x10C4, 0xEA60)]
        self.assertEqual(ff.pick_new_port({"COM3"}, ports), "COM7")
        self.assertEqual(ff.pick_new_port({"COM3", "COM7"}, ports), "COM9")
        self.assertIsNone(ff.pick_new_port({"COM7", "COM9"}, ports))
        self.assertEqual(ff.pick_new_port({"COM7", "COM9"}, ports, vid_pid=None), "COM3")

    def test_split_saved_log(self):
        esp, boot = ff.split_saved_log(fx("saved_ok.log"))
        self.assertEqual(ff.parse_esptool(esp)["hash_verified"], 4)
        self.assertEqual(ff.parse_boot_log(boot)["node_id"], 42)
        esp, boot = ff.split_saved_log(fx("boot_ok.log"))
        self.assertEqual(esp, "")

    def test_big_text(self):
        s = ff.big_text(42).splitlines()
        self.assertEqual(len(s), 5)
        self.assertEqual(s[0], "#  #    ###")  # "4" の1行目 + 字間2 + "2" の1行目

    def test_csv_row_roundtrip(self):
        e = ff.parse_esptool(fx("esptool_write_ok.txt"))
        b = ff.parse_boot_log(fx("boot_ok.log"))
        row = ff.make_row("2026-11-15T10:00:00", "COM7", "0001", e, b, "PASS", [], {}, "logs/x.log")
        self.assertEqual(set(row), set(ff.CSV_FIELDS))
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "first_flash.csv")
            ff.append_row(p, row)
            ff.append_row(p, row)
            rows = ff.read_rows(p)
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["mac"], "02:00:00:00:00:2a")
        self.assertEqual(rows[0]["node_id"], "42")
        self.assertEqual(rows[0]["id_from_mac"], "42")
        self.assertEqual(rows[0]["psram_mb"], "8")
        self.assertEqual(rows[0]["largest_after_wasm"], "98304")
        self.assertEqual(rows[0]["flash_detected"], "8MB")


class Cli(unittest.TestCase):
    def run_cli(self, *a):
        env = dict(os.environ, PYTHONIOENCODING="utf-8")
        return subprocess.run([sys.executable, os.path.join(os.path.dirname(HERE), "first_flash.py"), *a],
                              capture_output=True, text=True, encoding="utf-8", env=env)

    def test_parse_log_saved(self):
        r = self.run_cli("--parse-log", os.path.join(FIX, "saved_ok.log"))
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("PASS", r.stdout)

    def test_parse_log_boot_only_fail(self):
        r = self.run_cli("--parse-log", os.path.join(FIX, "boot_selftest_fail.log"))
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        self.assertIn("[selftest]", r.stdout)


if __name__ == "__main__":
    unittest.main()
