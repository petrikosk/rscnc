//! 2D tool radius correction, G40/G41/G42 (the `SMC_ToolCorr` analogue).
//!
//! Consecutive moves with an active correction mode form a *chain*; every
//! chain element is offset by the tool radius to the programmed side, outer
//! corners get a transition arc around the original corner point, inner
//! corners are trimmed to the exact intersection of the offset elements
//! (line/line, line/arc, arc/arc). A straight lead-in/lead-out line connects
//! the uncorrected positions before and after the chain.
//!
//! A contour detail the tool does not fit into — a slot narrower than the
//! tool, a step lower than its radius, an inside arc smaller than the tool —
//! would turn its offset elements inside out. Such elements are removed
//! (*loop removal*): the offset contour is closed across the detail by
//! intersecting the elements that survive on its two sides, transition arcs
//! included, so the tool passes the detail without cutting into it. A
//! junction is only accepted when it keeps the tool clear of the contour
//! elements around it — at the junction itself and all along the offset
//! element that leads up to it — and when no such junction exists the
//! contour is refused with an error rather than cut into.
//!
//! The corrected contour starts and ends square to the first and the last
//! contour element, and stands square to the contour at every outer or
//! tangent corner it reaches uncut. These points are taken as programmed:
//! where the tool cuts into another part of the contour at one of them — at
//! the two ends of a closed contour, typically — the path is emitted as it
//! always was. Program a lead-in and a lead-out move to avoid that.
//!
//! Limitations: correction is planar (helical arcs and splines are rejected)
//! and all chain moves must lie in one common work plane. Loop removal is
//! local: it takes out up to 256 consecutive offset elements, the first and
//! the last element of a corrected contour are never removed, and the tool
//! is checked against the eight contour elements before and after the ones
//! being joined — two distant parts of a contour that come closer to each
//! other than the tool diameter (a bottleneck) are not detected.

use super::Preprocessor;
use crate::error::PreprocessError;
use crate::gcode::TrcMode;
use crate::geometry::{min_opt, ArcSeg, GeoElem, GeoMove, Geometry, LineSeg, Segment, SwitchPoint};
use crate::queue::GeoQueue;
use crate::types::{PathPoint, WorkPlane};

const STAGE: &str = "ToolRadiusCorrection";
const EPS: f64 = 1e-9;

/// Most offset elements (transition arcs included) that loop removal takes
/// out between two surviving ones.
const MAX_REMOVED: usize = 256;

/// Contour elements before and after the ones a junction joins that the
/// tool must stay clear of as well.
const NEIGHBOURHOOD: usize = 8;

/// Tool radius correction preprocessor: offsets G41/G42 move chains by the
/// tool radius (see the module docs for the offsetting rules).
#[derive(Debug, Clone)]
pub struct ToolRadiusCorrection {
    /// Tool radius (internal units).
    pub tool_radius: f64,
    /// Remove the offset elements of contour details the tool does not fit
    /// into (the default). When `false`, such a detail is an error instead.
    pub remove_loops: bool,
}

impl ToolRadiusCorrection {
    /// Correction with the given tool radius (internal units), with loop
    /// removal.
    pub fn new(tool_radius: f64) -> Self {
        Self {
            tool_radius,
            remove_loops: true,
        }
    }

    /// Report a contour detail smaller than the tool as an error rather than
    /// removing it from the corrected path (builder style).
    pub fn without_loop_removal(mut self) -> Self {
        self.remove_loops = false;
        self
    }
}

/// 2D vector helpers in the active plane.
#[derive(Debug, Clone, Copy, PartialEq)]
struct V2 {
    x: f64,
    y: f64,
}

impl V2 {
    fn of(p: &PathPoint, plane: WorkPlane) -> Self {
        let (i0, i1, _) = plane.axes();
        V2 { x: p[i0], y: p[i1] }
    }

    fn to3(self, plane: WorkPlane, normal_coord: f64) -> PathPoint {
        let (i0, i1, i2) = plane.axes();
        let mut p = [0.0; 3];
        p[i0] = self.x;
        p[i1] = self.y;
        p[i2] = normal_coord;
        p
    }

    fn sub(self, o: V2) -> V2 {
        V2 {
            x: self.x - o.x,
            y: self.y - o.y,
        }
    }

    fn add(self, o: V2) -> V2 {
        V2 {
            x: self.x + o.x,
            y: self.y + o.y,
        }
    }

    fn scale(self, k: f64) -> V2 {
        V2 {
            x: self.x * k,
            y: self.y * k,
        }
    }

    fn dot(self, o: V2) -> f64 {
        self.x * o.x + self.y * o.y
    }

    fn cross(self, o: V2) -> f64 {
        self.x * o.y - self.y * o.x
    }

    fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    fn left(self) -> V2 {
        V2 {
            x: -self.y,
            y: self.x,
        }
    }

    fn dist(self, o: V2) -> f64 {
        self.sub(o).norm()
    }
}

/// A chain element being corrected, tagged with its source index in the chain.
struct Tagged {
    src: usize,
    mv: GeoMove,
}

/// One element of the raw offset contour: an offset chain element, or the
/// transition arc of an outer corner.
struct Raw {
    /// Chain element it belongs to (a transition arc: the one before it).
    src: usize,
    mv: GeoMove,
    /// First and last chain element it was derived from (a transition arc:
    /// the two that meet at its corner).
    lo: usize,
    hi: usize,
    /// The element ends exactly where the next raw element starts.
    connected: bool,
    /// An inside arc smaller than the tool: it has no offset (`mv` is the
    /// programmed element) and is never part of the corrected contour.
    vanished: bool,
}

/// A raw element that is (so far) part of the corrected contour, with the
/// path parameters and points it is cut to.
struct Kept {
    idx: usize,
    /// First chain element of the kept element before it (its own, for the
    /// first one): from there on the contour is checked against it.
    from: usize,
    start: f64,
    start_pt: Option<V2>,
    end: f64,
    end_pt: Option<V2>,
}

impl Kept {
    /// Where the element starts, when that is not a cut: the start of the
    /// corrected contour, or an outer or tangent corner.
    fn given_start(&self, raw: &Raw, plane: WorkPlane) -> Option<V2> {
        self.start_pt
            .is_none()
            .then(|| V2::of(&raw.mv.geo.start(), plane))
    }
}

/// Where two raw elements are joined: the point (`None`: they already
/// meet end to start) and its path parameter on either element.
struct Junction {
    pt: Option<V2>,
    s_prev: f64,
    s_next: f64,
}

/// What the junction rules need to know about the chain being corrected.
struct ChainCtx<'a> {
    chain: &'a [GeoMove],
    plane: WorkPlane,
    /// Closest the tool centre may come to a contour element: the tool
    /// radius less a rounding allowance.
    clearance: f64,
}

impl ChainCtx<'_> {
    /// `true` when a tool centred at `p` does not cut into the chain
    /// elements `lo..=hi` or their neighbourhood.
    fn clear(&self, p: V2, lo: usize, hi: usize) -> bool {
        let from = lo.saturating_sub(NEIGHBOURHOOD);
        let to = (hi + NEIGHBOURHOOD).min(self.chain.len() - 1);
        self.chain[from..=to]
            .iter()
            .all(|mv| distance_to(&mv.geo, p, self.plane) >= self.clearance)
    }

    /// The same for a tool that follows the offset element `geo` from the
    /// path parameter `from` to `to`.
    ///
    /// `given` are the ends of the stretch that are not for the correction
    /// to choose: where the corrected contour starts and ends, and where an
    /// element runs up to an outer or tangent corner uncut. They lie square
    /// to the programmed contour there, so an element the tool cuts into at
    /// such a point (the other end of a closed contour, typically) is the
    /// program's own doing and not held against the stretch.
    fn clear_along(
        &self,
        geo: &Geometry,
        (from, to): (f64, f64),
        (lo, hi): (usize, usize),
        given: [Option<V2>; 2],
    ) -> bool {
        let first = lo.saturating_sub(NEIGHBOURHOOD);
        let last = (hi + NEIGHBOURHOOD).min(self.chain.len() - 1);
        self.chain[first..=last].iter().all(|mv| {
            let programmed = |p: &V2| distance_to(&mv.geo, *p, self.plane) < self.clearance;
            given.iter().flatten().any(programmed) || self.clear_of(geo, from, to, &mv.geo)
        })
    }

    /// `true` when a tool following `geo` from `from` to `to` does not cut
    /// into the contour element `elem`.
    fn clear_of(&self, geo: &Geometry, from: f64, to: f64, elem: &Geometry) -> bool {
        let plane = self.plane;
        let at = |s: f64| V2::of(&geo.point_at(s), plane);
        let far = |p: V2| distance_to(elem, p, plane) >= self.clearance;
        // Nowhere near: no point of the stretch is further from its middle
        // than half its length.
        let half = 0.5 * (to - from);
        if distance_to(elem, at(from + half), plane) >= half + self.clearance {
            return true;
        }
        // The two cross
        if elem.length() > EPS
            && intersections(geo, elem, plane).into_iter().any(|p| {
                locate(elem, p, plane).is_some()
                    && locate(geo, p, plane).is_some_and(|s| s >= from && s <= to)
            })
        {
            return false;
        }
        // Otherwise they are nearest at an end of either, or where the line
        // between them is normal to both: opposite a circle centre.
        let own_centre = match geo {
            Geometry::Arc(a) => Some(centre(a, plane)),
            _ => None,
        };
        let mut opposite = [
            Some(V2::of(&elem.start(), plane)),
            Some(V2::of(&elem.end(), plane)),
            None,
            None,
        ];
        match (elem, own_centre) {
            (Geometry::Arc(e), Some(c)) => {
                let other = centre(e, plane);
                opposite[2] = Some(other);
                opposite[3] = Some(c.add(c.sub(other)));
            }
            (Geometry::Arc(e), None) => opposite[2] = Some(centre(e, plane)),
            (_, Some(c)) => opposite[2] = Some(nearest_on(elem, c, plane)),
            (_, None) => {}
        }
        far(at(from))
            && far(at(to))
            && opposite
                .into_iter()
                .flatten()
                .all(|q| foot(geo, q, plane).is_none_or(|s| s < from || s > to || far(at(s))))
    }
}

impl Preprocessor for ToolRadiusCorrection {
    fn name(&self) -> &'static str {
        STAGE
    }

    fn process(&mut self, queue: GeoQueue) -> Result<GeoQueue, PreprocessError> {
        let mut out: Vec<GeoElem> = Vec::with_capacity(queue.len());
        let mut elems = queue.into_iter().peekable();
        while let Some(elem) = elems.next() {
            let mode = match &elem {
                GeoElem::Move(m) if m.trc != TrcMode::Off => m.trc,
                _ => {
                    out.push(elem);
                    continue;
                }
            };
            // Collect the chain: the moves with the same mode. Everything
            // else in between — other queue elements and moves that only
            // drive auxiliary axes — is re-emitted after the corrected move
            // it follows, tagged with the number of chain moves before it.
            let mut chain: Vec<GeoMove> = Vec::new();
            let mut inter: Vec<(usize, GeoElem)> = Vec::new();
            let mut next = Some(elem);
            while let Some(e) = next.take() {
                match e {
                    GeoElem::Move(m) if !is_stationary(&m) => chain.push(m),
                    other => inter.push((chain.len(), other)),
                }
                next = elems.next_if(|e| !matches!(e, GeoElem::Move(m) if m.trc != mode));
            }
            let corrected = if chain.is_empty() {
                Vec::new()
            } else {
                self.correct_chain(&chain, mode)?
            };
            // Emit per source move: the elements interleaved before it, then
            // its corrected pieces (which are ordered by source move).
            let mut inter = inter.into_iter().peekable();
            let mut corrected = corrected.into_iter().peekable();
            // End point of the corrected path so far
            let mut at: Option<PathPoint> = None;
            // Auxiliary-axis positions there. A move that follows a removed
            // contour detail takes over the auxiliary travel that was
            // programmed on the detail, so the axes stay continuous.
            let mut aux_at: Option<Vec<f64>> = None;
            let mut follow = |m: &mut GeoMove| {
                if let Some(aux) = aux_at.take() {
                    if aux.len() == m.aux_start.len() {
                        m.aux_start = aux;
                    }
                }
                aux_at = Some(m.aux_end.clone());
            };
            for k in 0..=chain.len() {
                while let Some((_, e)) = inter.next_if(|(pos, _)| *pos == k) {
                    out.push(match (e, at) {
                        // A stationary move happens where the corrected
                        // path is, not at the programmed contour point.
                        (GeoElem::Move(mut m), Some(p)) => {
                            m.geo = Geometry::Line(LineSeg::new(p, p));
                            follow(&mut m);
                            GeoElem::Move(m)
                        }
                        (GeoElem::Move(mut m), None) => {
                            follow(&mut m);
                            GeoElem::Move(m)
                        }
                        (e, _) => e,
                    });
                }
                while let Some(mut t) = corrected.next_if(|t| t.src == k) {
                    at = Some(t.mv.geo.end());
                    follow(&mut t.mv);
                    out.push(GeoElem::Move(t.mv));
                }
            }
        }
        Ok(out.into_iter().collect())
    }
}

/// A move without path geometry (it only drives auxiliary axes).
fn is_stationary(mv: &GeoMove) -> bool {
    matches!(&mv.geo, Geometry::Line(l) if l.length() < EPS)
}

impl ToolRadiusCorrection {
    fn correct_chain(
        &self,
        chain: &[GeoMove],
        mode: TrcMode,
    ) -> Result<Vec<Tagged>, PreprocessError> {
        let first = &chain[0];
        let line0 = first.src_line;
        let gouge = |line: usize, msg: String| PreprocessError::new(STAGE, line, msg);

        let plane = self.common_plane(chain)?;
        let (_, _, i2) = plane.axes();
        let normal_coord = first.geo.start()[i2];
        let side = match mode {
            TrcMode::Left => 1.0,
            TrcMode::Right => -1.0,
            TrcMode::Off => unreachable!(),
        };
        let rt = self.tool_radius;

        // 1. Offset every element
        let first_real = chain.iter().position(|mv| !is_stationary(mv));
        let last_real = chain.iter().rposition(|mv| !is_stationary(mv));
        let mut off: Vec<(Tagged, bool)> = Vec::with_capacity(chain.len());
        for (k, mv) in chain.iter().enumerate() {
            let geo = match &mv.geo {
                Geometry::Line(l) => {
                    let d = V2::of(&l.p1, plane).sub(V2::of(&l.p0, plane));
                    let len = d.norm();
                    if len < EPS {
                        continue; // pure aux move inside a chain: nothing to offset
                    }
                    let shift = d.scale(1.0 / len).left().scale(side * rt);
                    Geometry::Line(LineSeg::new(
                        V2::of(&l.p0, plane).add(shift).to3(plane, normal_coord),
                        V2::of(&l.p1, plane).add(shift).to3(plane, normal_coord),
                    ))
                }
                Geometry::Arc(a) => {
                    if a.helix_delta.abs() > EPS {
                        return Err(gouge(
                            mv.src_line,
                            "tool radius correction does not support helical arcs".into(),
                        ));
                    }
                    let dir = if a.sweep > 0.0 { 1.0 } else { -1.0 };
                    let dr = -rt * side * dir;
                    let r_start = a.r_start + dr;
                    let r_end = a.r_end + dr;
                    if (r_start <= EPS || r_end <= EPS)
                        && self.remove_loops
                        && first_real.is_some_and(|f| k > f)
                        && last_real.is_some_and(|l| k < l)
                    {
                        // An inside arc smaller than the tool has no offset
                        // element at all: loop removal joins what is around it.
                        let vanished = Tagged {
                            src: k,
                            mv: mv.clone(),
                        };
                        off.push((vanished, true));
                        continue;
                    }
                    if r_start <= EPS || r_end <= EPS {
                        return Err(gouge(
                            mv.src_line,
                            format!(
                                "tool radius {rt} gouges an arc of radius {} (offset radius would be ≤ 0)",
                                a.r_start
                            ),
                        ));
                    }
                    Geometry::Arc(ArcSeg {
                        r_start,
                        r_end,
                        ..a.clone()
                    })
                }
                Geometry::Spline(_) => {
                    return Err(gouge(
                        mv.src_line,
                        "tool radius correction does not support spline segments".into(),
                    ));
                }
            };
            // The offset element runs parallel to the programmed one but is a
            // different length; a switch point keeps its relative place on it.
            let (old_len, new_len) = (mv.geo.length(), geo.length());
            let mut offset_mv = GeoMove { geo, ..mv.clone() };
            offset_mv.scale_switch_points(old_len, new_len);
            let offset = Tagged {
                src: k,
                mv: offset_mv,
            };
            off.push((offset, false));
        }
        if off.is_empty() {
            return Ok(Vec::new());
        }

        // 2. The raw offset contour: the offset elements in order, with a
        // transition arc around the original corner point at every outer
        // corner. Consecutive raw elements either meet end to start already,
        // or have to be cut to an intersection (inner corners).
        let mut raw: Vec<Raw> = Vec::with_capacity(off.len() * 2);
        for (next, vanished) in off {
            if let Some(prev) = raw.last_mut() {
                let prev_src = prev.src;
                let corner_src = &chain[prev_src];
                let corner = V2::of(&corner_src.geo.end(), plane);
                let t1 = tangent2(&prev.mv.geo, plane, true);
                let t2 = tangent2(&next.mv.geo, plane, false);
                let cross = t1.cross(t2);
                // Where the tool centre stands square to either element at
                // the corner (a vanished arc has no offset to take it from)
                let square = |t: V2| corner.add(t.left().scale(side * rt));
                let e1_end = if prev.vanished {
                    square(t1)
                } else {
                    V2::of(&prev.mv.geo.end(), plane)
                };
                let e2_start = if vanished {
                    square(t2)
                } else {
                    V2::of(&next.mv.geo.start(), plane)
                };

                if e1_end.dist(e2_start) < EPS {
                    // tangent-continuous: nothing to do
                    prev.connected = !prev.vanished && !vanished;
                } else if cross * side < -EPS || (cross.abs() <= EPS && t1.dot(t2) < 0.0) {
                    // Outer corner: transition arc around the original corner
                    // point. An exact reversal is the limit of an outer corner
                    // from either side: the tool goes round the tip in a half
                    // circle.
                    let ccw = mode == TrcMode::Right;
                    let arc = ArcSeg::from_center(
                        plane,
                        e1_end.to3(plane, normal_coord),
                        e2_start.to3(plane, normal_coord),
                        corner.to3(plane, normal_coord),
                        ccw,
                        1e-6 + rt * 1e-9,
                    )
                    .map_err(|e| gouge(corner_src.src_line, e))?;
                    prev.connected = !prev.vanished;
                    raw.push(Raw {
                        src: prev_src,
                        mv: GeoMove {
                            geo: Geometry::Arc(arc),
                            feed: corner_src.feed.min(chain[next.src].feed),
                            is_rapid: false,
                            aux_start: corner_src.aux_end.clone(),
                            aux_end: corner_src.aux_end.clone(),
                            trc: mode,
                            accel: min_opt(corner_src.accel, chain[next.src].accel),
                            decel: min_opt(corner_src.decel, chain[next.src].decel),
                            switch_points: Vec::new(),
                            src_line: corner_src.src_line,
                            obj_no: corner_src.obj_no,
                        },
                        lo: prev_src,
                        hi: next.src,
                        connected: !vanished,
                        vanished: false,
                    });
                }
                // (otherwise an inner corner, or near-colinear with a gap)
            }
            raw.push(Raw {
                src: next.src,
                lo: next.src,
                hi: next.src,
                mv: next.mv,
                connected: false,
                vanished,
            });
        }

        // 3. Join the raw elements. An element whose junction with the next
        // one would lie before its own start is inside out — the contour
        // detail it follows is smaller than the tool — as is one that does
        // not meet its neighbour at all. Loop removal then looks for the
        // nearest pair of elements, one already kept and one still to come,
        // that do have a valid junction, and drops what lies between them.
        // Arc radii that differ between start and end (within the decoder's
        // tolerance) blur the distances by as much.
        let mismatch = chain
            .iter()
            .filter_map(|mv| match &mv.geo {
                Geometry::Arc(a) => Some((a.r_end - a.r_start).abs()),
                _ => None,
            })
            .fold(0.0, f64::max);
        let cx = ChainCtx {
            chain,
            plane,
            clearance: rt - (1e-6 * (1.0 + rt) + mismatch),
        };
        let whole = |idx: usize, from: usize, raw: &[Raw]| Kept {
            idx,
            from,
            start: 0.0,
            start_pt: None,
            end: raw[idx].mv.geo.length(),
            end_pt: None,
        };
        let too_small = |line: usize| {
            gouge(
                line,
                format!(
                    "tool radius {rt} too large: no collision-free path past a contour detail smaller than the tool"
                ),
            )
        };
        let mut kept: Vec<Kept> = vec![whole(0, raw[0].lo, &raw)];
        let mut n = 1;
        while n < raw.len() {
            let mut hit = self
                .junction(&cx, &raw, kept.last().unwrap(), n)
                .map(|j| (0, n, j));
            if hit.is_none() && self.remove_loops {
                let most = (kept.len() - 1 + raw.len() - 1 - n).min(MAX_REMOVED);
                'search: for total in 1..=most {
                    for pops in 0..=total.min(kept.len() - 1) {
                        let j = n + (total - pops);
                        if j >= raw.len() {
                            continue;
                        }
                        let from = &kept[kept.len() - 1 - pops];
                        if let Some(found) = self.junction(&cx, &raw, from, j) {
                            hit = Some((pops, j, found));
                            break 'search;
                        }
                    }
                }
            }
            let Some((pops, j, found)) = hit else {
                let line = chain[raw[n].src].src_line;
                return Err(if self.remove_loops {
                    too_small(line)
                } else {
                    gouge(
                        line,
                        format!(
                            "tool radius {rt} too large: offset contours do not intersect at an inner corner (contour detail smaller than tool radius)"
                        ),
                    )
                });
            };
            kept.truncate(kept.len() - pops);
            let top = kept.last_mut().unwrap();
            top.end = found.s_prev;
            top.end_pt = found.pt;
            let from = raw[top.idx].lo;
            kept.push(Kept {
                start: found.s_next,
                start_pt: found.pt,
                ..whole(j, from, &raw)
            });
            n = j + 1;
        }
        // The rest of the last element is all that has not been checked yet
        let tail = kept.last().unwrap();
        let tail_raw = &raw[tail.idx];
        if self.remove_loops
            && !cx.clear_along(
                &tail_raw.mv.geo,
                (tail.start, tail.end),
                (tail.from, tail_raw.hi),
                [
                    tail.given_start(tail_raw, plane),
                    Some(V2::of(&tail_raw.mv.geo.end(), plane)),
                ],
            )
        {
            return Err(too_small(tail_raw.mv.src_line));
        }

        // 4. Cut the surviving elements to their junctions
        let mut raw: Vec<Option<Raw>> = raw.into_iter().map(Some).collect();
        let mut joined: Vec<Tagged> = Vec::with_capacity(kept.len());
        // Switch points of elements that are not part of the corrected path
        let mut carried: Vec<SwitchPoint> = Vec::new();
        // Something was left out since the last element that was emitted
        let mut gap = false;
        let mut next_raw = 0;
        for k in &kept {
            for removed in raw[next_raw..k.idx].iter_mut() {
                carried.append(&mut removed.take().unwrap().mv.switch_points);
                gap = true;
            }
            next_raw = k.idx + 1;
            let Raw { src, mut mv, .. } = raw[k.idx].take().unwrap();
            let src_line = mv.src_line;
            let len = mv.geo.length();
            let mut keep = 1e-9 * (1.0 + len);
            if let Geometry::Arc(a) = &mv.geo {
                keep = keep.max(1e-9 * a.r_start.max(a.r_end));
            }
            if k.end - k.start <= keep {
                // Cut down to nothing
                carried.append(&mut mv.switch_points);
                gap = true;
                continue;
            }
            if let Some(pt) = k.end_pt {
                trim_end(&mut mv.geo, pt, plane, normal_coord).map_err(|e| gouge(src_line, e))?;
            }
            if let Some(pt) = k.start_pt {
                trim_start(&mut mv.geo, pt, plane, normal_coord).map_err(|e| gouge(src_line, e))?;
            }
            if gap {
                // Start exactly where the path so far ends (an element cut
                // down to nothing leaves a rounding-sized step otherwise)
                if let Some(prev) = joined.last() {
                    let end = V2::of(&prev.mv.geo.end(), plane);
                    let step = end.dist(V2::of(&mv.geo.start(), plane));
                    if step > 0.0 && step < 1e-6 {
                        trim_start(&mut mv.geo, end, plane, normal_coord)
                            .map_err(|e| gouge(src_line, e))?;
                    }
                }
            }
            let new_len = mv.geo.length();
            mv.shift_switch_points(k.start, new_len);
            if !carried.is_empty() {
                // Switch points of removed elements fire at the junction
                for sp in &mut carried {
                    sp.position = 0.0;
                }
                mv.switch_points.splice(0..0, carried.drain(..));
            }
            joined.push(Tagged { src, mv });
            gap = false;
        }
        let Some(last) = joined.last_mut() else {
            return Err(gouge(
                line0,
                format!("tool radius {rt} too large: nothing is left of the corrected contour"),
            ));
        };
        if !carried.is_empty() {
            let end = last.mv.geo.length();
            for sp in &mut carried {
                sp.position = end;
            }
            last.mv.switch_points.append(&mut carried);
        }

        // 5. Lead-in / lead-out lines between uncorrected and corrected path
        let mut result: Vec<Tagged> = Vec::with_capacity(joined.len() + 2);
        let chain_start = first.geo.start();
        let off_start = joined.first().unwrap().mv.geo.start();
        if V2::of(&chain_start, plane).dist(V2::of(&off_start, plane)) > EPS {
            result.push(Tagged {
                src: 0,
                mv: GeoMove {
                    geo: Geometry::Line(LineSeg::new(chain_start, off_start)),
                    feed: first.feed,
                    is_rapid: false,
                    aux_start: first.aux_start.clone(),
                    aux_end: first.aux_start.clone(),
                    trc: mode,
                    accel: first.accel,
                    decel: first.decel,
                    switch_points: Vec::new(),
                    src_line: line0,
                    obj_no: first.obj_no,
                },
            });
        }
        result.extend(joined);
        let last_src = chain.len() - 1;
        let chain_end = chain[last_src].geo.end();
        let off_end = result.last().unwrap().mv.geo.end();
        if V2::of(&chain_end, plane).dist(V2::of(&off_end, plane)) > EPS {
            result.push(Tagged {
                src: last_src,
                mv: GeoMove {
                    geo: Geometry::Line(LineSeg::new(off_end, chain_end)),
                    feed: chain[last_src].feed,
                    is_rapid: false,
                    aux_start: chain[last_src].aux_end.clone(),
                    aux_end: chain[last_src].aux_end.clone(),
                    trc: mode,
                    accel: chain[last_src].accel,
                    decel: chain[last_src].decel,
                    switch_points: Vec::new(),
                    src_line: chain[last_src].src_line,
                    obj_no: chain[last_src].obj_no,
                },
            });
        }
        Ok(result)
    }

    /// The junction of a kept element with the raw element `j`, if they have
    /// a valid one: not before the kept element's own start, and with the
    /// tool clear of the contour elements between and around the two, both
    /// at the junction and on its way there along the kept element.
    fn junction(&self, cx: &ChainCtx, raw: &[Raw], prev: &Kept, j: usize) -> Option<Junction> {
        let (a, b) = (&raw[prev.idx], &raw[j]);
        if b.vanished {
            return None;
        }
        let adjacent = j == prev.idx + 1;
        let reachable = |s_prev: f64, given_end: Option<V2>| {
            let given = [prev.given_start(a, cx.plane), given_end];
            !self.remove_loops
                || cx.clear_along(&a.mv.geo, (prev.start, s_prev), (prev.from, b.hi), given)
        };
        if adjacent && a.connected {
            let s_prev = a.mv.geo.length();
            let end = V2::of(&a.mv.geo.end(), cx.plane);
            return reachable(s_prev, Some(end)).then_some(Junction {
                pt: None,
                s_prev,
                s_next: 0.0,
            });
        }
        let slack = 1e-9 * (1.0 + a.mv.geo.length());
        let corner = V2::of(&cx.chain[a.src].geo.end(), cx.plane);
        let mut best: Option<(f64, Junction)> = None;
        for p in intersections(&a.mv.geo, &b.mv.geo, cx.plane) {
            let (Some(s_prev), Some(s_next)) = (
                locate(&a.mv.geo, p, cx.plane),
                locate(&b.mv.geo, p, cx.plane),
            ) else {
                continue;
            };
            if s_prev < prev.start - slack {
                continue; // the element would run backward
            }
            let s_prev = s_prev.max(prev.start);
            if self.remove_loops && !cx.clear(p, a.lo, b.hi) {
                continue;
            }
            // Neighbours meet nearest to their original corner; across a
            // removed detail the first crossing along the kept element counts.
            let rank = if adjacent { p.dist(corner) } else { s_prev };
            if best.as_ref().is_none_or(|(r, _)| rank < *r) && reachable(s_prev, None) {
                best = Some((
                    rank,
                    Junction {
                        pt: Some(p),
                        s_prev,
                        s_next,
                    },
                ));
            }
        }
        best.map(|(_, junction)| junction)
    }

    /// The single work plane all chain elements must lie in.
    fn common_plane(&self, chain: &[GeoMove]) -> Result<WorkPlane, PreprocessError> {
        let mut candidates = vec![WorkPlane::Xy, WorkPlane::Zx, WorkPlane::Yz];
        for mv in chain {
            match &mv.geo {
                Geometry::Arc(a) => candidates.retain(|p| *p == a.plane),
                Geometry::Line(l) => candidates.retain(|p| {
                    let (_, _, n) = p.axes();
                    (l.p1[n] - l.p0[n]).abs() < EPS
                }),
                Geometry::Spline(_) => {
                    return Err(PreprocessError::new(
                        STAGE,
                        mv.src_line,
                        "tool radius correction does not support spline segments",
                    ));
                }
            }
        }
        candidates.first().copied().ok_or_else(|| {
            PreprocessError::new(
                STAGE,
                chain[0].src_line,
                "tool radius correction requires all corrected moves in one common work plane",
            )
        })
    }
}

/// In-plane unit tangent at the end (`at_end`) or start of a geometry.
fn tangent2(geo: &Geometry, plane: WorkPlane, at_end: bool) -> V2 {
    let s = if at_end { geo.length() } else { 0.0 };
    let t = geo.tangent_at(s);
    let (i0, i1, _) = plane.axes();
    V2 { x: t[i0], y: t[i1] }
}

/// All intersections of the curves two offset elements lie on (the full
/// line or circle; see [`locate`] for which of them are on the elements).
fn intersections(a: &Geometry, b: &Geometry, plane: WorkPlane) -> Vec<V2> {
    match (a, b) {
        (Geometry::Line(l1), Geometry::Line(l2)) => line_line(l1, l2, plane),
        (Geometry::Line(l), Geometry::Arc(arc)) => line_circle(l, arc, plane),
        (Geometry::Arc(arc), Geometry::Line(l)) => line_circle(l, arc, plane),
        (Geometry::Arc(a1), Geometry::Arc(a2)) => circle_circle(a1, a2, plane),
        // (spline chains are refused before any intersection is looked for)
        (Geometry::Spline(_), _) | (_, Geometry::Spline(_)) => Vec::new(),
    }
}

/// Path parameter (distance from the element's start) of a point on the
/// element's line or circle; `None` when the point is not on the element or
/// its modest extension (trim points may sit fractionally outside due to
/// rounding).
fn locate(geo: &Geometry, p: V2, plane: WorkPlane) -> Option<f64> {
    match geo {
        Geometry::Line(l) => {
            let p0 = V2::of(&l.p0, plane);
            let p1 = V2::of(&l.p1, plane);
            let d = p1.sub(p0);
            let len = d.norm();
            if len < EPS {
                return None;
            }
            let t = p.sub(p0).dot(d) / (len * len);
            (-1e-6..=1.0 + 1e-6)
                .contains(&t)
                .then(|| t.clamp(0.0, 1.0) * len)
        }
        Geometry::Arc(a) => {
            let (i0, i1, _) = plane.axes();
            let c = V2 {
                x: a.center[i0],
                y: a.center[i1],
            };
            const TAU: f64 = core::f64::consts::TAU;
            let ang = (p.y - c.y).atan2(p.x - c.x);
            // Angle travelled from the start, in the arc's own direction
            let span = a.sweep.abs();
            let mut delta = ((ang - a.theta0) * a.sweep.signum()).rem_euclid(TAU);
            if delta > span + 1e-6 {
                if delta < TAU - 1e-6 {
                    return None;
                }
                delta = 0.0; // a hair before the start
            }
            Some((delta / span).min(1.0) * a.length())
        }
        Geometry::Spline(_) => None,
    }
}

/// In-plane centre of an arc.
fn centre(a: &ArcSeg, plane: WorkPlane) -> V2 {
    let (i0, i1, _) = plane.axes();
    V2 {
        x: a.center[i0],
        y: a.center[i1],
    }
}

/// Path parameter of the point of a line or arc element that `p` lies
/// squarely opposite (the foot of the normal through `p`), if there is one.
fn foot(geo: &Geometry, p: V2, plane: WorkPlane) -> Option<f64> {
    match geo {
        Geometry::Line(l) => {
            let p0 = V2::of(&l.p0, plane);
            let d = V2::of(&l.p1, plane).sub(p0);
            let len_sqr = d.dot(d);
            if len_sqr <= 0.0 {
                return None;
            }
            let t = p.sub(p0).dot(d) / len_sqr;
            (0.0..=1.0).contains(&t).then(|| t * l.length())
        }
        Geometry::Arc(a) => {
            const TAU: f64 = core::f64::consts::TAU;
            let c = centre(a, plane);
            let span = a.sweep.abs();
            let delta = ((p.y - c.y).atan2(p.x - c.x) - a.theta0) * a.sweep.signum();
            let delta = delta.rem_euclid(TAU);
            (p.dist(c) > 0.0 && delta <= span).then(|| delta / span * a.length())
        }
        Geometry::Spline(_) => None,
    }
}

/// The point of a line or arc element nearest to `p`, in the plane.
fn nearest_on(geo: &Geometry, p: V2, plane: WorkPlane) -> V2 {
    match geo {
        Geometry::Line(l) => {
            let p0 = V2::of(&l.p0, plane);
            let d = V2::of(&l.p1, plane).sub(p0);
            let len_sqr = d.dot(d);
            let t = if len_sqr > 0.0 {
                (p.sub(p0).dot(d) / len_sqr).clamp(0.0, 1.0)
            } else {
                0.0
            };
            p0.add(d.scale(t))
        }
        Geometry::Arc(a) => {
            let (i0, i1, _) = plane.axes();
            let c = V2 {
                x: a.center[i0],
                y: a.center[i1],
            };
            const TAU: f64 = core::f64::consts::TAU;
            let rho = p.dist(c);
            let ang = (p.y - c.y).atan2(p.x - c.x);
            let span = a.sweep.abs();
            let delta = ((ang - a.theta0) * a.sweep.signum()).rem_euclid(TAU);
            if rho > 0.0 && delta <= span {
                // Radially opposite a point of the arc
                let r = a.r_start + (a.r_end - a.r_start) * (delta / span);
                c.add(p.sub(c).scale(r / rho))
            } else {
                let (start, end) = (V2::of(&a.start(), plane), V2::of(&a.end(), plane));
                if p.dist(start) <= p.dist(end) {
                    start
                } else {
                    end
                }
            }
        }
        Geometry::Spline(s) => V2::of(&s.start(), plane),
    }
}

/// Distance from a point to a programmed contour element, in the plane.
fn distance_to(geo: &Geometry, p: V2, plane: WorkPlane) -> f64 {
    match geo {
        Geometry::Spline(_) => f64::INFINITY,
        _ => p.dist(nearest_on(geo, p, plane)),
    }
}

fn line_line(l1: &LineSeg, l2: &LineSeg, plane: WorkPlane) -> Vec<V2> {
    let p1 = V2::of(&l1.p0, plane);
    let d1 = V2::of(&l1.p1, plane).sub(p1);
    let p2 = V2::of(&l2.p0, plane);
    let d2 = V2::of(&l2.p1, plane).sub(p2);
    let denom = d1.cross(d2);
    if denom.abs() < 1e-12 {
        return Vec::new();
    }
    let t = p2.sub(p1).cross(d2) / denom;
    vec![p1.add(d1.scale(t))]
}

fn line_circle(l: &LineSeg, a: &ArcSeg, plane: WorkPlane) -> Vec<V2> {
    let (i0, i1, _) = plane.axes();
    let p0 = V2::of(&l.p0, plane);
    let d = V2::of(&l.p1, plane).sub(p0);
    let len = d.norm();
    if len < EPS {
        return Vec::new();
    }
    let u = d.scale(1.0 / len);
    let c = V2 {
        x: a.center[i0],
        y: a.center[i1],
    };
    let r = (a.r_start + a.r_end) / 2.0;
    let pc = c.sub(p0);
    let b = u.dot(pc);
    let disc = b * b - (pc.dot(pc) - r * r);
    if disc < 0.0 {
        return Vec::new();
    }
    let sq = disc.sqrt();
    vec![p0.add(u.scale(b - sq)), p0.add(u.scale(b + sq))]
}

fn circle_circle(a1: &ArcSeg, a2: &ArcSeg, plane: WorkPlane) -> Vec<V2> {
    let (i0, i1, _) = plane.axes();
    let c1 = V2 {
        x: a1.center[i0],
        y: a1.center[i1],
    };
    let c2 = V2 {
        x: a2.center[i0],
        y: a2.center[i1],
    };
    let r1 = (a1.r_start + a1.r_end) / 2.0;
    let r2 = (a2.r_start + a2.r_end) / 2.0;
    let d = c1.dist(c2);
    if d < EPS || d > r1 + r2 + EPS || d < (r1 - r2).abs() - EPS {
        return Vec::new();
    }
    let a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    let h2 = r1 * r1 - a * a;
    let h = h2.max(0.0).sqrt();
    let dir = c2.sub(c1).scale(1.0 / d);
    let mid = c1.add(dir.scale(a));
    let perp = dir.left().scale(h);
    vec![mid.add(perp), mid.sub(perp)]
}

/// Trim the end of a geometry to `pt` (which lies on it).
fn trim_end(geo: &mut Geometry, pt: V2, plane: WorkPlane, normal_coord: f64) -> Result<(), String> {
    match geo {
        Geometry::Line(l) => {
            l.p1 = pt.to3(plane, normal_coord);
            Ok(())
        }
        Geometry::Arc(a) => {
            let old_sweep = a.sweep;
            let new = ArcSeg::from_center(
                plane,
                a.start(),
                pt.to3(plane, normal_coord),
                a.center,
                a.sweep > 0.0,
                f64::INFINITY,
            )?;
            if new.sweep.abs() > old_sweep.abs() + 1e-6 {
                return Err("inner-corner trim would enlarge an arc (contour detail smaller than tool radius)".into());
            }
            *a = new;
            Ok(())
        }
        Geometry::Spline(_) => Err("spline segments cannot be trimmed".into()),
    }
}

/// Trim the start of a geometry to `pt` (which lies on it).
fn trim_start(
    geo: &mut Geometry,
    pt: V2,
    plane: WorkPlane,
    normal_coord: f64,
) -> Result<(), String> {
    match geo {
        Geometry::Line(l) => {
            l.p0 = pt.to3(plane, normal_coord);
            Ok(())
        }
        Geometry::Arc(a) => {
            let old_sweep = a.sweep;
            let new = ArcSeg::from_center(
                plane,
                pt.to3(plane, normal_coord),
                a.end(),
                a.center,
                a.sweep > 0.0,
                f64::INFINITY,
            )?;
            if new.sweep.abs() > old_sweep.abs() + 1e-6 {
                return Err("inner-corner trim would enlarge an arc (contour detail smaller than tool radius)".into());
            }
            *a = new;
            Ok(())
        }
        Geometry::Spline(_) => Err("spline segments cannot be trimmed".into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decoder::{Decoder, DecoderConfig};

    fn moves(q: &GeoQueue) -> Vec<&GeoMove> {
        q.iter()
            .filter_map(|e| match e {
                GeoElem::Move(m) => Some(m),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn square_inside_correction_left() {
        // CCW square, tool left (G41) = inside the contour, radius 1:
        // every corner is an inner corner → all joins are trims, no arcs.
        let src = "G41 G1 F600 X10\nY10\nX0\nY0\nG40\n";
        let q = Decoder::decode_source(DecoderConfig::default(), src).unwrap();
        let out = ToolRadiusCorrection::new(1.0).process(q).unwrap();
        let m = moves(&out);
        // lead-in + 4 trimmed offset lines + lead-out = 6
        assert_eq!(m.len(), 6, "got {} moves", m.len());
        assert!(m.iter().all(|m| matches!(m.geo, Geometry::Line(_))));
        // Tool left of +X travel is the +Y side: first offset line at y = +1,
        // trimmed at the inner corner to x = 9
        let l1 = &m[1];
        assert!(
            (l1.geo.start()[1] - 1.0).abs() < 1e-9,
            "y = {}",
            l1.geo.start()[1]
        );
        assert!(
            (l1.geo.end()[0] - 9.0).abs() < 1e-9,
            "x = {}",
            l1.geo.end()[0]
        );
        assert!((l1.geo.end()[1] - 1.0).abs() < 1e-9);
    }

    #[test]
    fn square_right_correction_gets_transition_arcs() {
        // CCW square with G42 (tool right) = outside: outer corners → arcs
        let src = "G42 G1 F600 X10\nY10\nX0\nY0\nG40\n";
        let q = Decoder::decode_source(DecoderConfig::default(), src).unwrap();
        let out = ToolRadiusCorrection::new(1.0).process(q).unwrap();
        let m = moves(&out);
        let arcs = m
            .iter()
            .filter(|m| matches!(m.geo, Geometry::Arc(_)))
            .count();
        assert_eq!(arcs, 3, "expected 3 transition arcs, got {arcs}");
        // Offset of the first +X move with tool right is at y = -1
        let first_line = m
            .iter()
            .find(|m| matches!(m.geo, Geometry::Line(_)) && m.geo.length() > 2.0)
            .unwrap();
        assert!((first_line.geo.start()[1] + 1.0).abs() < 1e-9);
        // Transition arcs have tool radius
        for mv in m.iter() {
            if let Geometry::Arc(a) = &mv.geo {
                assert!((a.r_start - 1.0).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn arc_offset_radius() {
        // Full-quadrant CCW arc radius 10 with G41 (left = inside): offset radius 9
        let src = "G41 G1 F600 X0 Y-10\nG3 X10 Y0 I0 J10\nG40\n";
        // start at origin, first move to (0,-10)... keep simple: line then arc
        let q = Decoder::decode_source(DecoderConfig::default(), src).unwrap();
        let out = ToolRadiusCorrection::new(1.0).process(q).unwrap();
        let arc = moves(&out)
            .into_iter()
            .find_map(|m| match &m.geo {
                Geometry::Arc(a) if a.r_start > 5.0 => Some(a.clone()),
                _ => None,
            })
            .expect("offset arc present");
        assert!((arc.r_start - 9.0).abs() < 1e-9, "r = {}", arc.r_start);
    }

    #[test]
    fn exact_reversal_goes_round_the_tip() {
        // Out and straight back: the offset lines lie on opposite sides of
        // the contour and are joined by a half circle around its tip.
        for (code, y_out) in [("G41", 1.0), ("G42", -1.0)] {
            let src = format!("{code} G1 F600 X10\nX0\nG40\n");
            let q = Decoder::decode_source(DecoderConfig::default(), &src).unwrap();
            let out = ToolRadiusCorrection::new(1.0).process(q).unwrap();
            let m = moves(&out);
            let arc = m
                .iter()
                .find_map(|m| match &m.geo {
                    Geometry::Arc(a) => Some(a),
                    _ => None,
                })
                .unwrap_or_else(|| panic!("{code}: no transition arc"));
            assert!((arc.sweep.abs() - core::f64::consts::PI).abs() < 1e-9);
            assert!((arc.r_start - 1.0).abs() < 1e-9);
            // around the tip, on its far side
            let mid = arc.point_at(arc.length() / 2.0);
            assert!(
                (mid[0] - 11.0).abs() < 1e-9 && mid[1].abs() < 1e-9,
                "{mid:?}"
            );
            assert!((arc.start()[1] - y_out).abs() < 1e-9);
            // the corrected path is continuous
            for w in m.windows(2) {
                let (a, b) = (w[0].geo.end(), w[1].geo.start());
                assert!((0..3).all(|i| (a[i] - b[i]).abs() < 1e-9), "{code}: gap");
            }
        }
    }

    #[test]
    fn gouge_detection() {
        // Tool radius 6 on an inside arc of radius 5 → offset radius ≤ 0
        let src = "G41 G1 F600 X0 Y-5\nG3 X5 Y0 I0 J5\nG40\n";
        let q = Decoder::decode_source(DecoderConfig::default(), src).unwrap();
        assert!(ToolRadiusCorrection::new(6.0).process(q).is_err());
    }

    // --- loop removal -----------------------------------------------------

    fn xy(p: &PathPoint) -> V2 {
        V2 { x: p[0], y: p[1] }
    }

    /// Corrected contour of a program (without the lead-in and lead-out
    /// lines), checked against the programmed one: continuous, and — being
    /// the offset curve — exactly one tool radius away from the contour
    /// everywhere, so nowhere cutting into it.
    fn checked(cfg: &DecoderConfig, src: &str, radius: f64) -> Vec<GeoMove> {
        let q = Decoder::decode_source(cfg.clone(), src).unwrap();
        let contour: Vec<V2> = moves(&q)
            .iter()
            .filter(|m| m.trc != TrcMode::Off && m.geo.length() > 0.0)
            .flat_map(|m| {
                let len = m.geo.length();
                (0..=4000).map(move |i| xy(&m.geo.point_at(len * i as f64 / 4000.0)))
            })
            .collect();
        let out = ToolRadiusCorrection::new(radius)
            .process(q)
            .unwrap_or_else(|e| panic!("{src:?}: {e}"));
        let all: Vec<GeoMove> = moves(&out).into_iter().cloned().collect();
        for w in all.windows(2) {
            let gap = xy(&w[0].geo.end()).dist(xy(&w[1].geo.start()));
            assert!(gap < 1e-9, "{src:?}: gap of {gap}");
            assert_eq!(w[0].aux_end, w[1].aux_start, "{src:?}: aux step");
        }
        let corrected: Vec<GeoMove> = all[1..all.len() - 1].to_vec();
        for mv in &corrected {
            let len = mv.geo.length();
            assert!(len > 1e-9, "{src:?}: empty element");
            for i in 0..=200 {
                let p = xy(&mv.geo.point_at(len * i as f64 / 200.0));
                let d = contour
                    .iter()
                    .map(|c| c.dist(p))
                    .fold(f64::INFINITY, f64::min);
                assert!(
                    d >= radius - 1e-6 && d <= radius + 1e-4,
                    "{src:?}: tool centre ({}, {}) is {d} from the contour",
                    p.x,
                    p.y
                );
            }
        }
        corrected
    }

    fn kinds(m: &[GeoMove]) -> String {
        m.iter()
            .map(|m| match m.geo {
                Geometry::Line(_) => 'L',
                Geometry::Arc(_) => 'A',
                _ => '?',
            })
            .collect()
    }

    /// The same program for G41 and, mirrored about the X axis, for G42.
    fn both_sides(body: &str) -> [(String, f64); 2] {
        let mirrored: String = body
            .split_inclusive(|c: char| c.is_whitespace())
            .map(|w| match w.strip_prefix('Y').or(w.strip_prefix('J')) {
                Some(rest) if rest.starts_with('-') => format!("{}{}", &w[..1], &rest[1..]),
                Some(rest) => format!("{}-{rest}", &w[..1]),
                None => match w.trim() {
                    "G2" => w.replace("G2", "G3"),
                    "G3" => w.replace("G3", "G2"),
                    _ => w.to_string(),
                },
            })
            .collect();
        [
            (format!("G41 G1 F600 {body}\nG40\n"), 1.0),
            (format!("G42 G1 F600 {mirrored}\nG40\n"), -1.0),
        ]
    }

    #[test]
    fn slot_narrower_than_the_tool_is_passed_over() {
        // A slot 1 wide and 5 deep, tool radius 1: the tool cannot enter. It
        // dips into the slot's mouth as far as the two edges let it.
        for (src, sign) in both_sides("X10\nY-5\nX11\nY0\nX20") {
            let m = checked(&DecoderConfig::default(), &src, 1.0);
            assert_eq!(kinds(&m), "LAAL", "{src:?}");
            // the two transition arcs meet above the middle of the slot
            let meet = xy(&m[1].geo.end());
            assert!((meet.x - 10.5).abs() < 1e-9, "{meet:?}");
            assert!((meet.y - sign * 0.75f64.sqrt()).abs() < 1e-9, "{meet:?}");
            assert!((xy(&m[0].geo.end()).x - 10.0).abs() < 1e-9);
            assert!((xy(&m[3].geo.start()).x - 11.0).abs() < 1e-9);
            // without loop removal the slot is an error
            let q = Decoder::decode_source(DecoderConfig::default(), &src).unwrap();
            let err = ToolRadiusCorrection::new(1.0)
                .without_loop_removal()
                .process(q)
                .unwrap_err();
            assert!(err.message.contains("smaller than tool radius"), "{err}");
        }
        // A slot the tool does fit into is followed all the way down.
        for (src, _) in both_sides("X10\nY-5\nX13\nY0\nX20") {
            let m = checked(&DecoderConfig::default(), &src, 1.0);
            assert_eq!(kinds(&m), "LALLLAL", "{src:?}");
        }
    }

    #[test]
    fn step_lower_than_the_tool_radius_is_bridged() {
        // An inside step of 0.5 followed by an outer corner: the wall's
        // offset has no room, the tool goes from the floor's offset straight
        // onto the arc around the step's upper edge.
        for (src, sign) in both_sides("X10\nY0.5\nX20") {
            let m = checked(&DecoderConfig::default(), &src, 1.0);
            assert_eq!(kinds(&m), "LAL", "{src:?}");
            let meet = xy(&m[0].geo.end());
            assert!((meet.x - (10.0 - 0.75f64.sqrt())).abs() < 1e-9, "{meet:?}");
            assert!((meet.y - sign).abs() < 1e-9, "{meet:?}");
            let q = Decoder::decode_source(DecoderConfig::default(), &src).unwrap();
            assert!(ToolRadiusCorrection::new(1.0)
                .without_loop_removal()
                .process(q)
                .is_err());
        }
    }

    #[test]
    fn inside_arc_smaller_than_the_tool_vanishes() {
        // A fillet of radius 0.5 in an inside corner, tool radius 1: the
        // corrected path is that of the sharp corner.
        for (src, sign) in both_sides("X10\nG3 X10.5 Y0.5 I0 J0.5\nG1 Y10") {
            let m = checked(&DecoderConfig::default(), &src, 1.0);
            assert_eq!(kinds(&m), "LL", "{src:?}");
            let meet = xy(&m[0].geo.end());
            assert!((meet.x - 9.5).abs() < 1e-9 && (meet.y - sign).abs() < 1e-9);
            let q = Decoder::decode_source(DecoderConfig::default(), &src).unwrap();
            let err = ToolRadiusCorrection::new(1.0)
                .without_loop_removal()
                .process(q)
                .unwrap_err();
            assert!(err.message.contains("gouges an arc"), "{err}");
        }
        // The same fillet as a polygon of 30 chords: all of them go.
        let mut body = String::from("X10");
        for k in 1..=30 {
            let th = (k as f64 * 3.0 - 90.0).to_radians();
            body += &format!(
                "\nX{:.9} Y{:.9}",
                10.0 + 0.5 * th.cos(),
                0.5 + 0.5 * th.sin()
            );
        }
        body += "\nY10";
        for (src, sign) in both_sides(&body) {
            let m = checked(&DecoderConfig::default(), &src, 1.0);
            assert_eq!(kinds(&m), "LL", "{src:?}");
            let meet = xy(&m[0].geo.end());
            assert!((meet.x - 9.5).abs() < 1e-6 && (meet.y - sign).abs() < 1e-9);
        }
        // A fillet the tool fits into keeps its (smaller) offset arc.
        for (src, _) in both_sides("X10\nG3 X12 Y2 I0 J2\nG1 Y10") {
            let m = checked(&DecoderConfig::default(), &src, 1.0);
            assert_eq!(kinds(&m), "LAL", "{src:?}");
        }
    }

    #[test]
    fn notch_between_two_lines_loses_both_flanks() {
        // A V notch 0.6 wide: neither flank's offset survives, the arcs
        // around the notch's two edges are joined directly.
        for (src, sign) in both_sides("X10\nX10.3 Y-2\nX10.6 Y0\nX20") {
            let m = checked(&DecoderConfig::default(), &src, 1.0);
            assert_eq!(kinds(&m), "LAAL", "{src:?}");
            let meet = xy(&m[1].geo.end());
            assert!((meet.x - 10.3).abs() < 1e-9, "{meet:?}");
            assert!((meet.y - sign * (1.0f64 - 0.09).sqrt()).abs() < 1e-9);
        }
    }

    #[test]
    fn removed_elements_hand_over_switch_points_and_aux_travel() {
        let cfg = DecoderConfig {
            aux_letters: vec!['A'],
            start_aux: vec![0.0],
            ..Default::default()
        };
        // switch points on both flanks of the notch, a dwell-free M function
        // between them, and auxiliary travel on every move
        let src = "G41 G1 F600 X10 A1 H1 O0.5\nX10.3 Y-2 A2 H5 O0.5\nM7\nX10.6 Y0 A3 H-5 O0.5\nX20 A4 H-1 O0.5\nG40\n";
        let m = checked(&cfg, src, 1.0);
        assert_eq!(kinds(&m), "LAAL");
        let all: Vec<(usize, i32, f64)> = m
            .iter()
            .enumerate()
            .flat_map(|(i, mv)| {
                mv.switch_points
                    .iter()
                    .map(move |s| (i, s.number, s.position))
            })
            .collect();
        // every switch point is still there, in programmed order
        assert_eq!(
            all.iter().map(|s| s.1).collect::<Vec<_>>(),
            vec![1, 5, -5, -1]
        );
        // those of the removed flanks fire at the junction: the start of
        // the first element after the one that leads up to it
        assert_eq!((all[1].0, all[1].2), (2, 0.0));
        assert_eq!((all[2].0, all[2].2), (2, 0.0));
        for mv in &m {
            for w in mv.switch_points.windows(2) {
                assert!(w[0].position <= w[1].position);
            }
        }
        // the auxiliary axis arrives where it was programmed to
        assert_eq!(m.last().unwrap().aux_end, vec![4.0]);
        // the M function between the removed flanks is still in the queue
        let q = Decoder::decode_source(cfg, src).unwrap();
        let out = ToolRadiusCorrection::new(1.0).process(q).unwrap();
        assert_eq!(
            out.iter()
                .filter(|e| matches!(e, GeoElem::Event(_)))
                .count(),
            1
        );
    }

    #[test]
    fn contour_too_narrow_everywhere_is_still_an_error() {
        // A pocket 0.5 wide that the path runs into and out of again: there
        // is nowhere for a tool of radius 1 to go.
        for (src, _) in both_sides("X10\nY0.5\nX0") {
            let q = Decoder::decode_source(DecoderConfig::default(), &src).unwrap();
            let err = ToolRadiusCorrection::new(1.0).process(q).unwrap_err();
            assert_eq!(err.stage, "ToolRadiusCorrection");
            assert!(err.message.contains("smaller than the tool"), "{err}");
        }
        // A detail at the very end of the contour cannot be bridged either:
        // there is nothing behind it to join.
        for (src, _) in both_sides("X10\nY0.5") {
            let q = Decoder::decode_source(DecoderConfig::default(), &src).unwrap();
            assert!(ToolRadiusCorrection::new(1.0).process(q).is_err());
        }
    }

    #[test]
    fn closed_contour_with_an_approach_move_is_left_as_programmed() {
        // The approach move ends at the corner the contour closes on, with
        // the tool outside the wall the last move runs along: a collision
        // of the program's own making, emitted as it is with or without
        // loop removal.
        let src = "G0 X-10 Y-10\nG41 G1 F600 X0 Y0\nX20\nY20\nX0\nY0\nG40 G1 X-10 Y-10\n";
        let ends = |trc: ToolRadiusCorrection| -> Vec<(PathPoint, PathPoint)> {
            let q = Decoder::decode_source(DecoderConfig::default(), src).unwrap();
            let out = { trc }.process(q).unwrap();
            moves(&out)
                .iter()
                .map(|m| (m.geo.start(), m.geo.end()))
                .collect()
        };
        let with = ends(ToolRadiusCorrection::new(4.0));
        assert_eq!(
            with,
            ends(ToolRadiusCorrection::new(4.0).without_loop_removal())
        );
        // rapid, lead-in, approach, arc round the corner, four sides,
        // lead-out, way out
        assert_eq!(with.len(), 10);
    }

    #[test]
    fn random_profiles_never_gouge() {
        // Profiles running in +X with details of the tool's own size, lines
        // and minor arcs mixed. Unless the profile crosses itself there is a
        // corrected path, it has no gaps, and it is the offset curve: the
        // tool centre is exactly one tool radius from the contour all along.
        let mut state = 0x2545_f491_4f6c_dd1du64;
        let mut rnd = move |lo: f64, hi: f64| {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            lo + (hi - lo) * ((state >> 11) as f64 / (1u64 << 53) as f64)
        };
        let radius = 1.0;
        let (mut corrected, mut shortened) = (0, 0);
        for _ in 0..400 {
            let mut body = String::from("X10\n");
            let (mut x, mut y) = (10.0f64, 0.0f64);
            let details = 3 + rnd(0.0, 10.0) as usize;
            for _ in 0..details {
                let (nx, ny) = (x + rnd(0.3, 2.5), rnd(-1.5, 1.5));
                let chord = (nx - x).hypot(ny - y);
                match rnd(0.0, 4.0) as usize {
                    0 => body += &format!("G2 X{nx:.3} Y{ny:.3} R{:.3}\n", chord * rnd(0.6, 2.0)),
                    1 => body += &format!("G3 X{nx:.3} Y{ny:.3} R{:.3}\n", chord * rnd(0.6, 2.0)),
                    _ => body += &format!("G1 X{nx:.3} Y{ny:.3}\n"),
                }
                (x, y) = (nx, ny);
            }
            body += &format!("G1 X{:.3} Y0\nX{:.3}", x + 3.0, x + 13.0);
            for (src, _) in both_sides(&body) {
                let q = Decoder::decode_source(DecoderConfig::default(), &src).unwrap();
                let contour: Vec<Geometry> = moves(&q)
                    .iter()
                    .filter(|m| m.trc != TrcMode::Off && m.geo.length() > 0.0)
                    .map(|m| m.geo.clone())
                    .collect();
                let plain = ToolRadiusCorrection::new(radius)
                    .without_loop_removal()
                    .process(q.clone());
                let out = match ToolRadiusCorrection::new(radius).process(q) {
                    Ok(out) => out,
                    Err(e) => {
                        // only a profile that crosses itself is refused
                        let mut crossing = false;
                        for (i, a) in contour.iter().enumerate() {
                            for b in &contour[i + 1..] {
                                for p in intersections(a, b, WorkPlane::Xy) {
                                    let on = locate(a, p, WorkPlane::Xy).is_some()
                                        && locate(b, p, WorkPlane::Xy).is_some();
                                    let shared =
                                        [a.start(), a.end()].iter().any(|e| xy(e).dist(p) < 1e-3);
                                    crossing |= on && !shared;
                                }
                            }
                        }
                        assert!(crossing, "{src:?}: {e}");
                        continue;
                    }
                };
                corrected += 1;
                if plain.is_err() {
                    shortened += 1;
                }
                let all = moves(&out);
                for w in all.windows(2) {
                    let gap = xy(&w[0].geo.end()).dist(xy(&w[1].geo.start()));
                    assert!(gap < 1e-9, "{src:?}: gap of {gap}");
                }
                for mv in &all[1..all.len() - 1] {
                    let len = mv.geo.length();
                    for i in 0..=100 {
                        let p = xy(&mv.geo.point_at(len * i as f64 / 100.0));
                        let d = contour
                            .iter()
                            .map(|c| distance_to(c, p, WorkPlane::Xy))
                            .fold(f64::INFINITY, f64::min);
                        assert!(
                            d >= radius - 1e-6 && d <= radius + 1e-6,
                            "{src:?}: tool centre ({}, {}) is {d} from the contour",
                            p.x,
                            p.y
                        );
                    }
                }
            }
        }
        // nearly all can be machined, half of them only with details left out
        assert!(corrected > 700, "only {corrected} of 800 corrected");
        assert!(shortened > 300, "only {shortened} with removed details");
    }

    #[test]
    fn locate_and_distance_helpers() {
        let plane = WorkPlane::Xy;
        let line = Geometry::Line(LineSeg::new([0.0; 3], [4.0, 0.0, 0.0]));
        assert_eq!(locate(&line, V2 { x: 1.0, y: 0.0 }, plane), Some(1.0));
        assert_eq!(locate(&line, V2 { x: 4.1, y: 0.0 }, plane), None);
        assert_eq!(distance_to(&line, V2 { x: 5.0, y: 0.0 }, plane), 1.0);
        assert_eq!(distance_to(&line, V2 { x: 2.0, y: -3.0 }, plane), 3.0);
        // quarter circle of radius 2 around the origin, clockwise from +Y to +X
        let arc = Geometry::Arc(
            ArcSeg::from_center(
                plane,
                [0.0, 2.0, 0.0],
                [2.0, 0.0, 0.0],
                [0.0; 3],
                false,
                1e-9,
            )
            .unwrap(),
        );
        let at = |deg: f64, r: f64| V2 {
            x: r * deg.to_radians().cos(),
            y: r * deg.to_radians().sin(),
        };
        let s = locate(&arc, at(60.0, 2.0), plane).unwrap();
        assert!((s - 2.0 * 30.0f64.to_radians()).abs() < 1e-12);
        // a hair before the start counts as the start, further out does not
        assert_eq!(locate(&arc, at(90.0 + 1e-8, 2.0), plane), Some(0.0));
        assert_eq!(locate(&arc, at(100.0, 2.0), plane), None);
        assert_eq!(locate(&arc, at(-10.0, 2.0), plane), None);
        assert!((distance_to(&arc, at(45.0, 3.5), plane) - 1.5).abs() < 1e-12);
        assert!((distance_to(&arc, at(45.0, 0.5), plane) - 1.5).abs() < 1e-12);
        // beside the arc: the nearer end point
        let d = distance_to(&arc, V2 { x: -1.0, y: 2.0 }, plane);
        assert!((d - 1.0).abs() < 1e-12);
    }
}
