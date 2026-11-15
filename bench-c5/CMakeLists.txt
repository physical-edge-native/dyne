cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bench_c5)

# ---- wasm-micro-runtime の version.h 改行正規化 (node-esp32/CMakeLists.txt と同じ対処) ----
# WAMR コンポーネントは configure のたびに core/version.h をソースツリー内へ再生成し、
# Windows では CRLF で書かれる。コンポーネントマネージャのハッシュ検証が
# 「managed_components が変更された」と止まるのを防ぐため、LF に戻す。
set(_wamr_version_h
    ${CMAKE_CURRENT_LIST_DIR}/managed_components/espressif__wasm-micro-runtime/core/version.h)
if(EXISTS ${_wamr_version_h})
    file(SIZE ${_wamr_version_h} _wamr_vh_disk)
    file(READ ${_wamr_version_h} _wamr_vh_content)
    string(LENGTH "${_wamr_vh_content}" _wamr_vh_mem)
    if(NOT _wamr_vh_disk EQUAL _wamr_vh_mem)
        idf_build_get_property(_bench_python PYTHON)
        execute_process(
            COMMAND ${_bench_python} -c
            "import sys;p=sys.argv[1];b=open(p,'rb').read();open(p,'wb').write(b.replace(b'\\r\\n',b'\\n'))"
            ${_wamr_version_h}
            RESULT_VARIABLE _bench_norm_rc)
        if(NOT _bench_norm_rc EQUAL 0)
            message(WARNING "bench-c5: version.h normalize failed (rc=${_bench_norm_rc})")
        endif()
    endif()
endif()
