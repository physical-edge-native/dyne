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
/// 境界条件: 値固定 (Dirichlet)。値は既定 0 (dyne_set_bc_value で変更可)
pub const BC_DIRICHLET: u32 = 0;
/// 境界条件: 界面 (隣接ノードと交換)
pub const BC_INTERFACE: u32 = 1;
// ---- F1 (発注3、2026-09-14): 未観測領域境界の設定情報化で追加した3種 ----
/// 境界条件: ゼロ勾配 (断熱端)。ゴースト点は鏡映 u_{-1} = u_1
pub const BC_ZERO_GRAD: u32 = 2;
/// 境界条件: 線形外挿。拡散系では端点が時間凍結する (理論ノートT8) — 選択時に警告
pub const BC_LINEAR_EXTRAP: u32 = 3;
/// 境界条件: 半無限透過境界 (理論ノートT10)。補助変数は transparent モジュール参照
pub const BC_TRANSPARENT: u32 = 4;

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
    // ---- F1 (発注3): 未観測領域境界の設定情報化で追加した任意エクスポート ----
    /// Dirichlet境界の値 (face, value)。既定 0.0。dyne_init の前に呼ぶ
    pub set_bc_value: Option<wasmi::TypedFunc<(u32, f64), ()>>,
    /// 透過境界の指数和係数 (j, decay, half, weight) を1項ずつ渡す。
    /// dyne_init をまたいで保持される (補助変数のみ init でリセット)
    pub transparent_coef: Option<wasmi::TypedFunc<(u32, f64, f64, f64), ()>>,
    /// 観測点ナッジング (発注8): (i, y, γ) → イノベーション d = y − u_i (適用前)
    pub observe: Option<wasmi::TypedFunc<(u32, f64, f64), f64>>,
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
            set_bc_value: instance
                .get_typed_func::<(u32, f64), ()>(&store, "dyne_set_bc_value")
                .ok(),
            transparent_coef: instance
                .get_typed_func::<(u32, f64, f64, f64), ()>(&store, "dyne_transparent_coef")
                .ok(),
            observe: instance
                .get_typed_func::<(u32, f64, f64), f64>(&store, "dyne_observe")
                .ok(),
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
