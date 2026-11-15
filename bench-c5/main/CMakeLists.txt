# 埋め込む素材は bench-c5/assets/ に置く (tools/bench_ref.py make と tools/make_aot.ps1 が作る)。
# AOT (事前コンパイル) を選んだときは .wasm の代わりに wamrc で作った .aot を埋め込む。
if(CONFIG_BENCH_USE_AOT)
    set(_actor_ext "aot")
else()
    set(_actor_ext "wasm")
endif()

set(_assets ${CMAKE_CURRENT_LIST_DIR}/../assets)
# 2次元の場のアクターの再生 (2026-11-11): 縮小版 wasm と dyne-sim2d の呼び出しの記録 (.rec)。
# AOT の方式でも .wasm (インタプリタ) で動かす。作り方は tools/interp_bench.py make。
# 形のアクターは速くする工夫の組み合わせごとに wasm と記録を持つ (2026-11-13、
# docs/2026-11-11-shape-speedup.md。名前の対応は main/bench.c の ASSET と interp_bench.py に合わせる)。
# sdf2d_base_small.wasm は工夫の前の縮小版 (比較用の固定の素材。ソースからは作り直せない)
set(_rec_files sdf2d_grid_actor_small.wasm sdf2d_base_small.wasm sdf2d_nostd_small.wasm
               sdf2d_f32_small.wasm wave2d_actor_small.wasm
               shape_n13.rec shape_n5.rec shape_sw2_n5.rec shape_f32_n5.rec
               shape_h5_n5.rec shape_p4h.rec wave_n5.rec)
set(_embed "${_assets}/mlp_actor_small.${_actor_ext}" "${_assets}/heat1d_actor_small.${_actor_ext}"
           "${_assets}/model_a_f64.dynm" "${_assets}/model_b_f64.dynm" "${_assets}/model_b_f32.dynm")
foreach(_f mlp_actor_small.${_actor_ext} heat1d_actor_small.${_actor_ext}
           model_a_f64.dynm model_b_f64.dynm model_b_f32.dynm ${_rec_files})
    if(NOT EXISTS ${_assets}/${_f})
        message(FATAL_ERROR "bench-c5: assets/${_f} がありません。README の手順 (bench_ref.py make / interp_bench.py make / make_aot.ps1) で作ってください")
    endif()
endforeach()
foreach(_f ${_rec_files})
    list(APPEND _embed "${_assets}/${_f}")
endforeach()

idf_component_register(
    SRCS "bench.c"
    INCLUDE_DIRS "."
    EMBED_FILES ${_embed}
    PRIV_REQUIRES esp_timer esp_hw_support heap
)
