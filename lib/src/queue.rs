//! The geometry queue (the `SMC_OUTQUEUE` analogue): decoded path elements
//! in program order, consumed by preprocessors and the interpolator.

use crate::geometry::GeoElem;
use std::collections::VecDeque;

#[derive(Debug, Clone, Default, PartialEq)]
pub struct GeoQueue {
    elems: VecDeque<GeoElem>,
}

impl GeoQueue {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, e: GeoElem) {
        self.elems.push_back(e);
    }

    pub fn pop_front(&mut self) -> Option<GeoElem> {
        self.elems.pop_front()
    }

    pub fn front(&self) -> Option<&GeoElem> {
        self.elems.front()
    }

    pub fn get(&self, i: usize) -> Option<&GeoElem> {
        self.elems.get(i)
    }

    pub fn get_mut(&mut self, i: usize) -> Option<&mut GeoElem> {
        self.elems.get_mut(i)
    }

    pub fn len(&self) -> usize {
        self.elems.len()
    }

    pub fn is_empty(&self) -> bool {
        self.elems.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &GeoElem> {
        self.elems.iter()
    }

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
