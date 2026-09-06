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
