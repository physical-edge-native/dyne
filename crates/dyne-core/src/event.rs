//! Event queue in virtual time.
//!
//! Ordering rule (needed for determinism): events at the same time are delivered in the
//! order in which they were scheduled. No wall clock is used.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// A queue of events of type `E` ordered by virtual time, then by scheduling order.
pub struct EventQueue<E> {
    /// (time, seq, ev_index)
    heap: BinaryHeap<Reverse<(u64, u64, usize)>>,
    events: Vec<Option<E>>,
    seq: u64,
}

impl<E> EventQueue<E> {
    /// An empty queue.
    pub fn new() -> Self {
        EventQueue { heap: BinaryHeap::new(), events: Vec::new(), seq: 0 }
    }

    /// Schedule `ev` at virtual time `t`.
    pub fn schedule(&mut self, t: u64, ev: E) {
        self.events.push(Some(ev));
        let idx = self.events.len() - 1;
        self.seq += 1;
        self.heap.push(Reverse((t, self.seq, idx)));
    }

    /// Take the earliest event (by time; events at the same time in scheduling order).
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
