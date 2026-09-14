//! dyne-core: ホスト側シミュレーション共通基盤
//!
//! 発注6・案B (docs/2026-09-02-swarm-port-options.md、2026-09-02 ユーザー決定) の
//! 切り出し。dyne-sim からモデル非依存の部品を移設したライブラリで、
//! dyne-sim (1次元熱伝導) と dyne-swarm-sim (群れ、実装予定) の両ホストが載る。
//!
//! - [`rng`] 決定論的乱数 (xorshift64*) と個体シードの導出規則
//! - [`event`] 離散イベント・仮想時刻のイベントキュー
//! - [`net`] 仮想ネットワーク (損失・遅延ジッタ・分断)
//! - [`actor`] WASMアクター (wasmi) の実行ラッパ (Dyne C ABI)
//! - [`wire`] ワイヤ形式v2 (dyne-wire の再エクスポート。単一定義)
//!
//! 移設の規律: dyne-sim の従来実装と**ビット互換** (乱数の消費順・イベントの
//! 順序決定を含めて変更しない)。切り出しの無変化は既存全回帰のビット一致で証明する。

pub mod actor;
pub mod event;
pub mod net;
pub mod rng;
pub mod transparent;

pub use dyne_wire as wire;
