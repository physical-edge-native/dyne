//! 群れアクター差込口の共有定数 (発注6 第2段、2026-09-04)
//!
//! ホスト (dyne-swarm-sim) とアクター (swarm-agent-actor) の双方が本モジュールに
//! 依存し、dyne_cfg のキーと個体規則の番号を別々に書かない (メッセージ形式と
//! 同じ「二重定義の禁止」の適用)。差込口の本体 (dyne_inbox_write /
//! dyne_outbox_read の意味論) は docs/2026-09-02-swarm-port-design.md §5。

/// dyne_cfg のキー: 個体規則 (値は MODE_*)
pub const CFG_MODE: u32 = 0;
/// dyne_cfg のキー: 最適温度 T_opt [℃]
pub const CFG_T_OPT: u32 = 1;
/// dyne_cfg のキー: 最適域の幅 tol (|T−T_opt| < tol)
pub const CFG_TOL: u32 = 2;
/// dyne_cfg のキー: 1ステップの最大歩幅
pub const CFG_SPEED: u32 = 3;
/// dyne_cfg のキー: センサ雑音の標準偏差
pub const CFG_NOISE: u32 = 4;
/// dyne_cfg のキー: 通信半径 (分離挙動の距離基準に使う)
pub const CFG_RADIUS: u32 = 5;
/// dyne_cfg のキー: 伝聞の最大鮮度 (ステップ数。超えたら捨てる)
pub const CFG_GOSSIP_MAX_AGE: u32 = 6;
/// dyne_cfg のキー: 個体シードの下位32ビット (f64で正確に表現できる)。
/// LO→HI の順で注入し、HI の書き込みで内蔵乱数が初期化される
pub const CFG_SEED_LO: u32 = 7;
/// dyne_cfg のキー: 個体シードの上位32ビット
pub const CFG_SEED_HI: u32 = 8;

/// 個体規則: 通信なしの対照群
pub const MODE_SOLO: u32 = 0;
/// 個体規則: 温度だけ共有し最良の隣へ寄る
pub const MODE_FOLLOW: u32 = 1;
/// 個体規則: 位置+温度を共有し局所勾配を合意
pub const MODE_CONSENSUS: u32 = 2;
/// 個体規則: consensus + 最良地点の伝聞 (既定・本命)
pub const MODE_GOSSIP: u32 = 3;

/// 2次元の場のアクター (heat2d-actor ほか) の差込口の共有定数 (2026-10-10、
/// 設計書 docs/2026-10-10-order10-design.md の2次元の土台)。ホスト (dyne-sim2d) と
/// アクターの双方が本モジュールに依存する (二重定義の禁止)。
pub mod field2d {
    /// 面ID: 西 (x の小さい側)
    pub const FACE_W: u32 = 0;
    /// 面ID: 東 (x の大きい側)
    pub const FACE_E: u32 = 1;
    /// 面ID: 南 (y の小さい側)
    pub const FACE_S: u32 = 2;
    /// 面ID: 北 (y の大きい側)
    pub const FACE_N: u32 = 3;

    /// 面の種類: 値固定 (Dirichlet)。その面の格子点を値に固定する
    pub const BC_DIRICHLET: u32 = 0;
    /// 面の種類: 界面 (隣接ノードと界面サンプル列を交換する)
    pub const BC_INTERFACE: u32 = 1;
    /// 面の種類: ゼロ勾配 (断熱端)。ゴースト点は鏡映
    pub const BC_ZERO_GRAD: u32 = 2;

    /// dyne_inbox_write の戻り値: 全て受理
    pub const INBOX_OK: i32 = 0;
    /// dyne_inbox_write の戻り値: 一部を受理しなかった (界面ID・ステップ・点の数の不一致。計数する)
    pub const INBOX_REJECTED: i32 = 1;
    /// dyne_inbox_write の戻り値: 枠またはメッセージの形式が不正 (それ以降は読まない)
    pub const INBOX_MALFORMED: i32 = -1;

    /// dyne_step の戻り値: 全界面のゴースト値が揃っていた
    pub const STEP_OK: i32 = 0;
    /// dyne_step の戻り値: 未受領の界面があった (前回のゴースト値のまま計算した)
    pub const STEP_MISSING_GHOST: i32 = 1;
}

/// 格子型アクター (sdf2d-grid-actor) の dyne_cfg のキー (2026-10-10、設計書
/// docs/2026-10-10-shape-grid-actor-design.md)。dyne_init2d の前に注入する。
/// 角度はラジアン、長さはメートル。
pub mod sdf2d {
    /// 格子の原点 (大域添字 (0,0) の点の部屋座標)
    pub const CFG_ORIGIN_X: u32 = 0;
    pub const CFG_ORIGIN_Y: u32 = 1;
    /// 切り詰め幅 μ
    pub const CFG_MU: u32 = 2;
    /// 統合の重みの上限
    pub const CFG_W_MAX: u32 = 3;
    /// 統合の遅れ L (ステップ数)。取得ステップ s の光線は全ノードが s+L で統合する
    pub const CFG_DELAY: u32 = 4;
    /// 距離の定義: 0 = 斜めの距離 (光線に沿った距離)、1 = 正面への垂直距離
    pub const CFG_RANGE_DEF: u32 = 5;
    /// 対象なしの区画を最大距離まで空きとして刻むか (0/1。既定0=刻まない)
    pub const CFG_CARVE_NO_TARGET: u32 = 6;
    /// センサの最大距離
    pub const CFG_MAX_RANGE: u32 = 7;
    /// センサの視野 (水平、全角)
    pub const CFG_FOV: u32 = 8;
    /// 測距フレームのうち使う行 (0始まり)
    pub const CFG_ZONE_ROW: u32 = 9;
    /// 以降の CFG_SENSOR_X/Y/YAW の対象にするセンサID (このノードに付いているセンサ)
    pub const CFG_SENSOR_SEL: u32 = 10;
    pub const CFG_SENSOR_X: u32 = 11;
    pub const CFG_SENSOR_Y: u32 = 12;
    /// センサの向き (x軸から反時計回り)
    pub const CFG_SENSOR_YAW: u32 = 13;

    /// 距離の定義の値
    pub const RANGE_DEF_RADIAL: u32 = 0;
    pub const RANGE_DEF_PERPENDICULAR: u32 = 1;

    /// 有効な状態値 (VL53L5CX の資料で距離が有効とされる 5 と 9。評価計画と同じ判定)
    pub fn status_valid(st: u32) -> bool {
        st == 5 || st == 9
    }

    /// dyne_get_aux の量の番号
    pub const AUX_D: u32 = 1;
    pub const AUX_W: u32 = 2;
}
