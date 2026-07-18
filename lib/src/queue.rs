//! The geometry queue (the `SMC_OUTQUEUE` analogue): decoded path elements
//! in program order, consumed by preprocessors and the interpolator.

use crate::geometry::GeoElem;
use std::collections::VecDeque;

/// FIFO of decoded path elements in program order.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GeoQueue {
    elems: VecDeque<GeoElem>,
}

impl GeoQueue {
    /// An empty queue.
    pub fn new() -> Self {
        Self::default()
    }

    /// Append an element at the back.
    pub fn push(&mut self, e: GeoElem) {
        self.elems.push_back(e);
    }

    /// Remove and return the front element.
    pub fn pop_front(&mut self) -> Option<GeoElem> {
        self.elems.pop_front()
    }

    /// The front element, if any.
    pub fn front(&self) -> Option<&GeoElem> {
        self.elems.front()
    }

    /// The element at index `i`, if any.
    pub fn get(&self, i: usize) -> Option<&GeoElem> {
        self.elems.get(i)
    }

    /// Mutable access to the element at index `i`, if any.
    pub fn get_mut(&mut self, i: usize) -> Option<&mut GeoElem> {
        self.elems.get_mut(i)
    }

    /// Number of elements in the queue.
    pub fn len(&self) -> usize {
        self.elems.len()
    }

    /// `true` when the queue has no elements.
    pub fn is_empty(&self) -> bool {
        self.elems.is_empty()
    }

    /// Iterate over the elements in program order.
    pub fn iter(&self) -> impl Iterator<Item = &GeoElem> {
        self.elems.iter()
    }

    /// Iterate mutably over the elements in program order.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut GeoElem> {
        self.elems.iter_mut()
    }

    /// Insert an element at index `i`, shifting later elements back.
    pub fn insert(&mut self, i: usize, e: GeoElem) {
        self.elems.insert(i, e);
    }

    /// Remove and return the element at index `i`.
    pub fn remove(&mut self, i: usize) -> Option<GeoElem> {
        self.elems.remove(i)
    }
}

impl FromIterator<GeoElem> for GeoQueue {
    fn from_iter<T: IntoIterator<Item = GeoElem>>(iter: T) -> Self {
        Self {
            elems: iter.into_iter().collect(),
        }
    }
}

impl IntoIterator for GeoQueue {
    type Item = GeoElem;
    type IntoIter = <VecDeque<GeoElem> as IntoIterator>::IntoIter;
    fn into_iter(self) -> Self::IntoIter {
        self.elems.into_iter()
    }
}
