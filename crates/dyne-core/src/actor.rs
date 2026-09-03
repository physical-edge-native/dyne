//! WASMアクター (wasmi) の実行ラッパ (dyne-sim 従来実装の移設)。
//!
//! Dyne C ABI (1次元場アクター) のエクスポート束。dyne_cfg 以降は任意
//! エクスポート (学習型アクターのみ)。既存の熱伝導アクターの差込口は
//! 当面不変 (2026-09-02 ユーザー決定。汎用メッセージバッファ方式への移行は
//! 2次元化の際)。群れアクターの差込口 (dyne_inbox_write / dyne_outbox_read) は
//! 群れ移植の段で本モジュールに追加する。

/// 面ID: 左
pub const FACE_LEFT: u32 = 0;
/// 面ID: 右
pub const FACE_RIGHT: u32 = 1;
/// 境界条件: 外部境界 (Dirichlet u=0)
pub const BC_DIRICHLET: u32 = 0;
/// 境界条件: 界面 (隣接ノードと交換)
pub const BC_INTERFACE: u32 = 1;

pub struct Actor {
    pub store: wasmi::Store<()>,
    pub init: wasmi::TypedFunc<(u32, f64, f64, f64, f64, u32, u32), i32>,
    pub step: wasmi::TypedFunc<(), ()>,
    pub bval: wasmi::TypedFunc<u32, f64>,
    pub bgrad: wasmi::TypedFunc<u32, f64>,
    pub set_neighbor: wasmi::TypedFunc<(u32, f64, f64), ()>,
    pub set: wasmi::TypedFunc<(u32, f64), ()>,
    pub get: wasmi::TypedFunc<u32, f64>,
    pub x: wasmi::TypedFunc<u32, f64>,
    pub len: wasmi::TypedFunc<(), u32>,
    // ---- 学習型アクター (heat1d-pinn-actor) の任意エクスポート。差分法アクターには無い ----
    /// ハイパーパラメータ設定 (dyne_init の前に呼ぶ)
    pub cfg: Option<wasmi::TypedFunc<(u32, f64), ()>>,
    /// 境界条件の切り替え (重みを保ったまま。無ければ再init+状態復元で代替)
    pub set_bc: Option<wasmi::TypedFunc<(u32, u32), ()>>,
    /// 隠れ層の幅変更 = GROW/SHRINK
    pub resize: Option<wasmi::TypedFunc<u32, i32>>,
    /// 現在のモデルを格子点で評価 (ステップを進めない。機能保存の検査用)
    pub eval: Option<wasmi::TypedFunc<u32, f64>>,
    /// wasm宣言の線形メモリ(ページ) — RAM上限チェック用
    pub memory_pages: u64,
}

impl Actor {
    pub fn load(engine: &wasmi::Engine, module: &wasmi::Module) -> Self {
        let mut store = wasmi::Store::new(engine, ());
        let linker: wasmi::Linker<()> = wasmi::Linker::new(engine);
        let instance = linker
            .instantiate(&mut store, module)
            .expect("instantiate")
            .start(&mut store)
            .expect("start");
        let memory_pages = instance
            .get_memory(&store, "memory")
            .map(|m| m.size(&store) as u64)
            .unwrap_or(0);
        macro_rules! func {
            ($name:literal, $p:ty, $r:ty) => {
                instance.get_typed_func::<$p, $r>(&store, $name).expect($name)
            };
        }
        Actor {
            init: func!("dyne_init", (u32, f64, f64, f64, f64, u32, u32), i32),
            step: func!("dyne_step", (), ()),
            bval: func!("dyne_boundary_value", u32, f64),
            bgrad: func!("dyne_boundary_grad", u32, f64),
            set_neighbor: func!("dyne_set_neighbor", (u32, f64, f64), ()),
            set: func!("dyne_set", (u32, f64), ()),
            get: func!("dyne_get", u32, f64),
            x: func!("dyne_x", u32, f64),
            len: func!("dyne_len", (), u32),
            cfg: instance.get_typed_func::<(u32, f64), ()>(&store, "dyne_cfg").ok(),
            set_bc: instance.get_typed_func::<(u32, u32), ()>(&store, "dyne_set_bc").ok(),
            resize: instance.get_typed_func::<u32, i32>(&store, "dyne_resize").ok(),
            eval: instance.get_typed_func::<u32, f64>(&store, "dyne_eval").ok(),
            store,
            memory_pages,
        }
    }

    /// load + ハイパーパラメータ適用 (学習型アクターのみ。差分法アクターでは無視)
    pub fn load_with(engine: &wasmi::Engine, module: &wasmi::Module, pinn_cfg: &[(u32, f64)]) -> Self {
        let mut a = Actor::load(engine, module);
        if let Some(f) = a.cfg {
            for &(k, v) in pinn_cfg {
                f.call(&mut a.store, (k, v)).unwrap();
            }
        }
        a
    }

    /// 学習型アクターか (dyne_cfg を持つ = heat1d-pinn-actor)
    pub fn is_learning(&self) -> bool {
        self.cfg.is_some()
    }
}
