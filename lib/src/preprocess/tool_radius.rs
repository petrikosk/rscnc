//! 2D tool radius correction, G40/G41/G42 (the `SMC_ToolCorr` analogue).
//!
//! Consecutive moves with an active correction mode form a *chain*; every
//! chain element is offset by the tool radius to the programmed side, outer
//! corners get a transition arc around the original corner point, inner
//! corners are trimmed to the exact intersection of the offset elements
//! (line/line, line/arc, arc/arc). A straight lead-in/lead-out line connects
//! the uncorrected positions before and after the chain.
//!
//! Limitations (v1): correction is planar (helical arcs are rejected), all
//! chain moves must lie in one common work plane, and a contour detail
//! smaller than the tool radius (an offset element that vanishes when
//! trimmed) is reported as an error rather than resolved by removing loops.

use super::Preprocessor;
use crate::error::PreprocessError;
use crate::gcode::TrcMode;
use crate::geometry::{ArcSeg, GeoElem, GeoMove, Geometry, LineSeg, Segment};
use crate::queue::GeoQueue;
use crate::types::{PathPoint, WorkPlane};

const STAGE: &str = "ToolRadiusCorrection";
const EPS: f64 = 1e-9;

/// Tool radius correction preprocessor: offsets G41/G42 move chains by the
/// tool radius (see the module docs for the offsetting rules).
#[derive(Debug, Clone)]
pub struct ToolRadiusCorrection {
    /// Tool radius (internal units).
    pub tool_radius: f64,
}

impl ToolRadiusCorrection {
    /// Correction with the given tool radius (internal units).
    pub fn new(tool_radius: f64) -> Self {
        Self { tool_radius }
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

impl Preprocessor for ToolRadiusCorrection {
    fn name(&self) -> &'static str {
        STAGE
    }

    fn process(&mut self, queue: GeoQueue) -> Result<GeoQueue, PreprocessError> {
        let elems: Vec<GeoElem> = queue.into_iter().collect();
        let mut out: Vec<GeoElem> = Vec::with_capacity(elems.len());
        let mut i = 0;
        while i < elems.len() {
            let mode = match &elems[i] {
                GeoElem::Move(m) if m.trc != TrcMode::Off => m.trc,
                other => {
                    out.push(other.clone());
                    i += 1;
                    continue;
                }
            };
            // Collect the chain: moves with the same mode; other queue elements
            // in between are re-emitted after the corrected move they follow.
            let mut chain: Vec<GeoMove> = Vec::new();
            let mut inter: Vec<(usize, GeoElem)> = Vec::new();
            while i < elems.len() {
                match &elems[i] {
                    GeoElem::Move(m) if m.trc == mode => {
                        chain.push(m.clone());
                        i += 1;
                    }
                    GeoElem::Move(_) => break,
                    other => {
                        inter.push((chain.len(), other.clone()));
                        i += 1;
                    }
                }
            }
            let corrected = self.correct_chain(&chain, mode)?;
            // Emit: elements interleaved at chain position 0 first, then per
            // source move its corrected pieces followed by its interleaved elements.
            for (pos, e) in inter.iter().filter(|(p, _)| *p == 0) {
                let _ = pos;
                out.push(e.clone());
            }
            for k in 0..chain.len() {
                for t in corrected.iter().filter(|t| t.src == k) {
                    out.push(GeoElem::Move(t.mv.clone()));
                }
                for (_, e) in inter.iter().filter(|(p, _)| *p == k + 1) {
                    out.push(e.clone());
                }
            }
        }
        Ok(out.into_iter().collect())
    }
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
        let mut off: Vec<Tagged> = Vec::with_capacity(chain.len() + 2);
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
            };
            off.push(Tagged {
                src: k,
                mv: GeoMove { geo, ..mv.clone() },
            });
        }
        if off.is_empty() {
            return Ok(Vec::new());
        }

        // 2. Join consecutive offset elements
        let mut joined: Vec<Tagged> = Vec::with_capacity(off.len() * 2);
        let mut iter = off.into_iter();
        joined.push(iter.next().unwrap());
        for mut next in iter {
            let prev_idx = joined.len() - 1;
            let prev_src = joined[prev_idx].src;
            let corner_src = &chain[prev_src];
            let corner = V2::of(&corner_src.geo.end(), plane);
            let t1 = tangent2(&joined[prev_idx].mv.geo, plane, true);
            let t2 = tangent2(&next.mv.geo, plane, false);
            let cross = t1.cross(t2);
            let e1_end = V2::of(&joined[prev_idx].mv.geo.end(), plane);
            let e2_start = V2::of(&next.mv.geo.start(), plane);

            if e1_end.dist(e2_start) < EPS {
                // tangent-continuous: nothing to do
            } else if cross * side < -EPS {
                // Outer corner: transition arc around the original corner point
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
                joined.push(Tagged {
                    src: prev_src,
                    mv: GeoMove {
                        geo: Geometry::Arc(arc),
                        feed: corner_src.feed.min(chain[next.src].feed),
                        is_rapid: false,
                        aux_start: corner_src.aux_end.clone(),
                        aux_end: corner_src.aux_end.clone(),
                        trc: mode,
                        src_line: corner_src.src_line,
                        obj_no: corner_src.obj_no,
                    },
                });
            } else {
                // Inner corner (or near-colinear with a gap): trim both
                // elements to their intersection
                let pt = intersect_near(&joined[prev_idx].mv.geo, &next.mv.geo, plane, corner)
                    .ok_or_else(|| {
                        gouge(
                            corner_src.src_line,
                            format!(
                                "tool radius {rt} too large: offset contours do not intersect at an inner corner"
                            ),
                        )
                    })?;
                let src_line = corner_src.src_line;
                trim_end(&mut joined[prev_idx].mv.geo, pt, plane, normal_coord)
                    .map_err(|e| gouge(src_line, e))?;
                trim_start(&mut next.mv.geo, pt, plane, normal_coord)
                    .map_err(|e| gouge(src_line, e))?;
            }
            joined.push(next);
        }

        // 3. Lead-in / lead-out lines between uncorrected and corrected path
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
                    src_line: chain[last_src].src_line,
                    obj_no: chain[last_src].obj_no,
                },
            });
        }
        Ok(result)
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

/// Intersection of two offset elements nearest to the original corner.
fn intersect_near(a: &Geometry, b: &Geometry, plane: WorkPlane, corner: V2) -> Option<V2> {
    let candidates = match (a, b) {
        (Geometry::Line(l1), Geometry::Line(l2)) => line_line(l1, l2, plane),
        (Geometry::Line(l), Geometry::Arc(arc)) => line_circle(l, arc, plane),
        (Geometry::Arc(arc), Geometry::Line(l)) => line_circle(l, arc, plane),
        (Geometry::Arc(a1), Geometry::Arc(a2)) => circle_circle(a1, a2, plane),
    };
    candidates
        .into_iter()
        .filter(|p| on_element(a, *p, plane) && on_element(b, *p, plane))
        .min_by(|p, q| {
            p.dist(corner)
                .partial_cmp(&q.dist(corner))
                .unwrap_or(core::cmp::Ordering::Equal)
        })
}

/// Whether the point lies on the element or its modest extension
/// (trim points may sit fractionally outside due to rounding).
fn on_element(geo: &Geometry, p: V2, plane: WorkPlane) -> bool {
    match geo {
        Geometry::Line(l) => {
            let p0 = V2::of(&l.p0, plane);
            let p1 = V2::of(&l.p1, plane);
            let d = p1.sub(p0);
            let len = d.norm();
            if len < EPS {
                return false;
            }
            let t = p.sub(p0).dot(d) / (len * len);
            (-1e-6..=1.0 + 1e-6).contains(&t)
        }
        Geometry::Arc(a) => {
            let (i0, i1, _) = plane.axes();
            let c = V2 {
                x: a.center[i0],
                y: a.center[i1],
            };
            let ang = (p.y - c.y).atan2(p.x - c.x);
            let mut delta = ang - a.theta0;
            const TAU: f64 = core::f64::consts::TAU;
            if a.sweep > 0.0 {
                while delta < 0.0 {
                    delta += TAU;
                }
                delta <= a.sweep + 1e-6
            } else {
                while delta > 0.0 {
                    delta -= TAU;
                }
                delta >= a.sweep - 1e-6
            }
        }
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
    fn gouge_detection() {
        // Tool radius 6 on an inside arc of radius 5 → offset radius ≤ 0
        let src = "G41 G1 F600 X0 Y-5\nG3 X5 Y0 I0 J5\nG40\n";
        let q = Decoder::decode_source(DecoderConfig::default(), src).unwrap();
        assert!(ToolRadiusCorrection::new(6.0).process(q).is_err());
    }
}
