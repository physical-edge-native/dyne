//! 離散イベント・仮想時刻のイベントキュー (dyne-sim 従来実装の移設)。
//!
//! 順序の決定則 (ビット互換の要): 同時刻のイベントは**スケジュールされた順**
//! (通し番号 seq の昇順) に配る。壁時計は使わない。

use std::cmp::Reverse;
use std::collections::BinaryHeap;

pub struct EventQueue<E> {
    /// (time, seq, ev_index)
    heap: BinaryHeap<Reverse<(u64, u64, usize)>>,
    events: Vec<Option<E>>,
    seq: u64,
}

impl<E> EventQueue<E> {
    pub fn new() -> Self {
        EventQueue { heap: BinaryHeap::new(), events: Vec::new(), seq: 0 }
    }

    /// 仮想時刻 t にイベントを予約する
    pub fn schedule(&mut self, t: u64, ev: E) {
        self.events.push(Some(ev));
        let idx = self.events.len() - 1;
        self.seq += 1;
        self.heap.push(Reverse((t, self.seq, idx)));
    }

    /// 最も早いイベントを取り出す (時刻昇順、同時刻は予約順)
    pub fn pop(&mut self) -> Option<(u64, E)> {
        self.heap.pop().map(|Reverse((t, _, idx))| {
            let ev = self.events[idx].take().expect("event consumed twice");
            (t, ev)
        })
    }
}

impl<E> Default for EventQueue<E> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn order_by_time_then_fifo() {
        let mut q = EventQueue::new();
        q.schedule(10, "b");
        q.schedule(5, "a");
        q.schedule(10, "c");
        assert_eq!(q.pop(), Some((5, "a")));
        assert_eq!(q.pop(), Some((10, "b"))); // 同時刻は予約順
        assert_eq!(q.pop(), Some((10, "c")));
        assert_eq!(q.pop(), None);
    }
}
