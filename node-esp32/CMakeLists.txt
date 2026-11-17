cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# 2026-11-17: WAMR が線形メモリの確保を「用途つき」で呼ぶようにする (main.c の wamr_malloc が
# 線形メモリだけを PSRAM に置くため)。全コンポーネントに同じ定義を渡す (WAMR の関数の型が変わるので、
# main と WAMR で食い違わないように)。ESP-IDF の WAMR の Kconfig にはこの項目が無い
idf_build_set_property(COMPILE_DEFINITIONS "WASM_MEM_ALLOC_WITH_USAGE=1" APPEND)
project(dyne_node)

# ---- wasm-micro-runtime の version.h 改行正規化 (2026-09-18 設計判断) ----
# WAMR コンポーネントの build-scripts/version.cmake は CMake configure のたびに
# core/version.h をソースツリー内へ configure_file で再生成する。Windows では
# configure_file が CRLF で書くため、パッケージ版 (LF、CHECKSUMS.json のハッシュ
# e261247b…) と24バイトだけ食い違い、コンポーネントマネージャのハッシュ検証が
# fullclean 等で「managed_components が変更された」と停止する (改行差のみで
# 内容は同一であることを検証済み)。ここで configure 完了後に LF へ正規化して
# パッケージ版とバイト一致に戻す。コンポーネント本体は一切編集しない
# (components/ への移動=11MB のベンダーツリー化を避けるための最小の対処。
# 経緯: docs/2026-09-14-order7-report.md §6.7)。
# 検出: file(SIZE) はバイト長、file(READ) はWindowsでCRLF→LF変換して読むため、
# 両者の差 = CRLF の個数。差があれば python (ESP-IDF環境に必ずある) でバイナリ
# 正規化する (CMake の file(WRITE) もテキストモードでCRLF化するため使えない)。
set(_wamr_version_h
    ${CMAKE_CURRENT_LIST_DIR}/managed_components/espressif__wasm-micro-runtime/core/version.h)
if(EXISTS ${_wamr_version_h})
    file(SIZE ${_wamr_version_h} _wamr_vh_disk)
    file(READ ${_wamr_version_h} _wamr_vh_content)
    string(LENGTH "${_wamr_vh_content}" _wamr_vh_mem)
    if(NOT _wamr_vh_disk EQUAL _wamr_vh_mem)
        idf_build_get_property(_dyne_python PYTHON)
        execute_process(
            COMMAND ${_dyne_python} -c
            "import sys;p=sys.argv[1];b=open(p,'rb').read();open(p,'wb').write(b.replace(b'\\r\\n',b'\\n'))"
            ${_wamr_version_h}
            RESULT_VARIABLE _dyne_norm_rc)
        if(_dyne_norm_rc EQUAL 0)
            message(STATUS "dyne: normalized wamr core/version.h to LF (managed hash check)")
        else()
            message(WARNING "dyne: version.h normalize failed (rc=${_dyne_norm_rc})")
        endif()
    endif()
endif()
