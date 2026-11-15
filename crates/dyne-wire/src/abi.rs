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
    /// 古い観測を忘れる仕組み (2026-11-11、docs/2026-11-11-dynamic-shape.md)。
    /// 既定値ではどれも働かず、従来とビット単位で同じ結果になる。
    /// 重みの減衰率 λ (1ステップごとに w ← λ·w。既定 1 = 減衰しない)
    pub const CFG_DECAY: u32 = 14;
    /// 減衰した重みがこれ未満になった格子点を未観測 (w = 0, d = 0) に戻す (既定 0 = 戻さない)
    pub const CFG_W_FORGET: u32 = 15;
    /// 食い違いの閾値 τ: 新しい観測と統合済みの値の差が τ を超えたら、統合済みの重みを
    /// CFG_DISAGREE_W まで下げてから統合する (既定 0 = 使わない)
    pub const CFG_DISAGREE: u32 = 16;
    /// 食い違いのときに残す重み (既定 1)
    pub const CFG_DISAGREE_W: u32 = 17;
    /// 距離の反復の間引き n: ステップ番号が n の倍数のときだけ反復する (既定 1 = 毎ステップ。
    /// 2026-11-11、docs/2026-11-11-shape-speedup.md。表面の近くの φ は観測 d そのものなので、
    /// 間引いて遅くなるのは表面から離れた点への距離の伝わり方だけ)
    pub const CFG_SWEEP_EVERY: u32 = 18;
    /// 減衰・忘却を視野の中の格子点だけに効かせる (0/1。既定 0 = 全格子点を毎ステップ減衰)。
    /// 1 なら、そのステップに統合する光線が通る格子点だけを、最初の統合の直前に1回減衰・忘却する。
    /// サーボで別の向きを見ている間に壁が消えないようにする (2026-11-13、docs/2026-11-11-shape-speedup.md)
    pub const CFG_DECAY_IN_VIEW: u32 = 19;
    /// CFG_DECAY_IN_VIEW = 1 のとき、視野の外の格子点に毎ステップ掛ける減衰率 (既定 1 = 保つ)。
    /// 1 未満なら、見えなくなった古い表面 (動かした物の、今は隠れた側の跡) もいずれ忘れる (2026-11-13)
    pub const CFG_DECAY_OUT: u32 = 20;

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

/// 2次元の波動方程式のアクター (wave2d-actor) の dyne_cfg のキー (2026-11-11、
/// docs/2026-11-11-wave2d.md)。dyne_init2d の前に注入する (init をまたいで保持)。
/// 差込口の残り (面・メールボックス・観測) は field2d と同じ。dyne_init2d の alpha は
/// 波の速さ c として解釈する。
pub mod wave2d {
    /// 初期条件の種類 (値は IC_*。既定 IC_SIN_MODE)。初速度はどれも0
    pub const CFG_IC: u32 = 0;
    /// 定在波のモード番号 m (x方向)、n (y方向)。既定 1, 1
    pub const CFG_MODE_M: u32 = 1;
    pub const CFG_MODE_N: u32 = 2;
    /// 振幅 (既定 1)
    pub const CFG_AMP: u32 = 3;
    /// ガウス型の山の中心 (x, y) と幅 σ (既定 0.5, 0.5, 0.05)
    pub const CFG_GX: u32 = 4;
    pub const CFG_GY: u32 = 5;
    pub const CFG_SIGMA: u32 = 6;
    /// 吸収境界にする外周の面のビット列 (bit f = 面ID f。既定 0 = なし)。界面の面では無視。
    /// 1次のエングクイスト・マイダ条件をムーアの方法で離散化したものを使う
    pub const CFG_ABSORB: u32 = 7;

    /// 初期条件: sin(mπx)·sin(nπy) (値固定0の正方形の定在波)
    pub const IC_SIN_MODE: u32 = 0;
    /// 初期条件: ガウス型の山 A·exp(−((x−gx)²+(y−gy)²)/(2σ²))
    pub const IC_GAUSS: u32 = 1;
    /// 初期条件: cos(mπx)·cos(nπy) (ゼロ勾配の正方形の定在波)
    pub const IC_COS_MODE: u32 = 2;
}

/// IMU の値を隣との通信だけで融合するアクター (imu-fusion-actor) の差込口の共有定数
/// (2026-11-13、docs/2026-11-13-imu-fusion.md)。差込口は field2d と同じ一式
/// (dyne_cfg → dyne_set_face → dyne_init2d、メールボックス、dyne_step、dyne_len/dyne_get) に、
/// 標本を入れる dyne_imu_push と待ち合わせ用の dyne_imu_need_until_us を足したもの。
/// 隣は「界面」の面 (field2d::BC_INTERFACE) で表す (直線・環・格子は1ノードあたり隣4つまで)。
/// dyne_init2d(nx, ny, ix0, iy0, h, alpha, dt) は dt (合意の1反復の周期 [s]) だけを使う。
pub mod imu {
    /// dyne_cfg のキー: 1区間のステップ数 M (区間 = 標本を平均する時間窓。1区間 = M·dt。既定 10)
    pub const CFG_EPOCH_STEPS: u32 = 0;
    /// dyne_cfg のキー: 標本の区間への割り当て方 (値は TIME_*。既定 TIME_STAMP)
    pub const CFG_TIME_MODE: u32 = 1;
    /// dyne_cfg のキー: ステップ0の時刻 t0 [µs] (同期した時刻。f64 で整数を正確に運べる範囲)。既定 0
    pub const CFG_T0_US: u32 = 2;
    /// dyne_cfg のキー: 標本の時刻印に足すずれ [µs] (実験で時計をわざとずらすため。既定 0)
    pub const CFG_TS_OFFSET_US: u32 = 3;
    /// dyne_cfg のキー: 相補フィルタの係数 α (角度 = α·(前の角度 + 角速度·区間) + (1−α)·加速度からの角度。既定 0.9)
    pub const CFG_CF_ALPHA: u32 = 4;
    /// dyne_cfg のキー: 角速度のゼロ点の較正に使う最初の区間の数 (その間は板を静止させる。既定 0 = 較正しない)
    pub const CFG_GYRO_CAL_EPOCHS: u32 = 5;
    /// dyne_cfg のキー: 合意の始め方 (値は CONS_*。既定 CONS_DYNAMIC)
    pub const CFG_CONSENSUS: u32 = 6;

    /// 合意の始め方: 区間ごとに状態を自分の和で置き直す (区間内の M 回の反復だけで平均に近づける)
    pub const CONS_RESET: u32 = 0;
    /// 合意の始め方: 動的合意平均。状態に「今の区間の和 − 前の区間の和」だけを足す。
    /// 全ノードの状態の和は常に今の区間の和の総和に等しく、変わらない成分 (ゼロ点のずれ) の
    /// 食い違いは区間をまたいで混ざり続けて消える。隣の値が届かなかった反復があると和の誤差が残り続ける
    pub const CONS_DYNAMIC: u32 = 1;

    /// 割り当て方: 時刻印で区間を決める (区間 e は (t0 + e·E, t0 + (e+1)·E]、E = M·dt)
    pub const TIME_STAMP: u32 = 0;
    /// 割り当て方: 時刻印を使わず、届いた順 (前の区間を閉じた後に入った標本を今の区間に入れる)
    pub const TIME_ARRIVAL: u32 = 1;

    /// 合意平均で交換する界面サンプル列の1点あたりの量の数:
    /// [次数 (隣の数), 合意の状態 x (重み, 重み×平均 ax, ay, az, gx, gy, gz)] (count = 1)。
    /// 重みは区間に標本があったノードで 1、無ければ 0 (ノードごとに等しい重みで平均する)
    pub const PER_POINT: u32 = 8;

    /// dyne_imu_push の戻り値: 受理
    pub const PUSH_OK: i32 = 0;
    /// dyne_imu_push の戻り値: その区間はもう閉じていた (遅れた標本。捨てて計数)
    pub const PUSH_LATE: i32 = 1;
    /// dyne_imu_push の戻り値: 先の区間すぎる (ため置きの外。捨てて計数)
    pub const PUSH_EARLY: i32 = 2;
    /// dyne_imu_push の戻り値: dyne_init2d の前
    pub const PUSH_NOT_READY: i32 = -1;

    /// dyne_get の番号 (dyne_len = G_LEN)。角度はラジアン、加速度 m/s²、角速度 rad/s
    /// 最後に融合を終えた区間の番号 (まだ無ければ −1)
    pub const G_EPOCH: u32 = 0;
    /// 標本のあったノードの割合 (合意の状態の重みの平均。全ノードに標本があれば 1 に収束する)
    pub const G_FUSED_N: u32 = 1;
    /// 融合した ax, ay, az, gx, gy, gz (6個続く)
    pub const G_FUSED: u32 = 2;
    pub const G_FUSED_ROLL: u32 = 8;
    pub const G_FUSED_PITCH: u32 = 9;
    /// 自分だけの標本数・平均 (6個)・角度 (同じ区間。比較用)
    pub const G_LOCAL_N: u32 = 10;
    pub const G_LOCAL: u32 = 11;
    pub const G_LOCAL_ROLL: u32 = 17;
    pub const G_LOCAL_PITCH: u32 = 18;
    /// 計数: 遅れた標本・先すぎる標本・届かなかった隣の値
    pub const G_LATE: u32 = 19;
    pub const G_EARLY: u32 = 20;
    pub const G_MISSING: u32 = 21;
    pub const G_LEN: u32 = 22;
}
