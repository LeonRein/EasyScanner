//! The known world of a synthetic scan: the printed board on its z = 0 plane
//! and simple solids standing on it, all in the OpenCV board frame (mm). The
//! camera looks at the printed face from −Z, so "up" (toward the camera,
//! where objects stand) is −Z, as in [`crate::geom::Pose::up_sign`].

use serde::{Deserialize, Serialize};

use crate::config::BoardSpec;
use crate::geom::{V3, add, dot, norm, scale, sub};

/// A solid whose surface the scan should reproduce.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Shape {
    Sphere {
        center: V3,
        radius: f64,
    },
    /// Axis-aligned box between two corners.
    Cuboid {
        min: V3,
        max: V3,
    },
}

impl Shape {
    /// A sphere of radius `r` lying on the board at `(x, y)`.
    pub fn ball_on_board(x: f64, y: f64, r: f64) -> Shape {
        Shape::Sphere { center: [x, y, -r], radius: r }
    }

    /// A `w`×`d` block of height `h` standing on the board, corner at `(x, y)`.
    pub fn block_on_board(x: f64, y: f64, w: f64, d: f64, h: f64) -> Shape {
        Shape::Cuboid { min: [x, y, -h], max: [x + w, y + d, 0.0] }
    }

    /// Nearest ray parameter `s > eps` where `o + s·dir` (dir unit) enters
    /// or leaves the solid.
    pub fn hit(&self, o: V3, dir: V3) -> Option<f64> {
        const EPS: f64 = 1e-7;
        match *self {
            Shape::Sphere { center, radius } => {
                let oc = sub(o, center);
                let b = dot(oc, dir);
                let c = dot(oc, oc) - radius * radius;
                let disc = b * b - c;
                if disc < 0.0 {
                    return None;
                }
                let sq = disc.sqrt();
                [-b - sq, -b + sq].into_iter().find(|&s| s > EPS)
            }
            Shape::Cuboid { min, max } => {
                let (mut t0, mut t1) = (f64::NEG_INFINITY, f64::INFINITY);
                for i in 0..3 {
                    if dir[i].abs() < 1e-15 {
                        if o[i] < min[i] || o[i] > max[i] {
                            return None;
                        }
                        continue;
                    }
                    let (a, b) = ((min[i] - o[i]) / dir[i], (max[i] - o[i]) / dir[i]);
                    t0 = t0.max(a.min(b));
                    t1 = t1.min(a.max(b));
                }
                if t0 > t1 {
                    return None;
                }
                [t0, t1].into_iter().find(|&s| s > EPS)
            }
        }
    }

    /// Signed distance from the surface: negative inside.
    pub fn signed_distance(&self, p: V3) -> f64 {
        match *self {
            Shape::Sphere { center, radius } => norm(sub(p, center)) - radius,
            Shape::Cuboid { min, max } => {
                let c = scale(add(min, max), 0.5);
                let h = scale(sub(max, min), 0.5);
                let q: V3 = std::array::from_fn(|i| (p[i] - c[i]).abs() - h[i]);
                let outside = norm(q.map(|v| v.max(0.0)));
                outside + q[0].max(q[1]).max(q[2]).min(0.0)
            }
        }
    }
}

/// What a ray hit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Surface {
    /// The board plane (inside the printed area or the table around it).
    Board,
    Shape(usize),
}

/// The printed ChArUco board's geometry (OpenCV layout: square (0,0) black,
/// markers in the white squares).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct BoardDims {
    pub squares_x: u32,
    pub squares_y: u32,
    pub square_mm: f64,
    pub marker_mm: f64,
}

impl Default for BoardDims {
    /// The app's default board (`BoardSpec::default`): 8x8, 36 mm.
    fn default() -> Self {
        BoardDims { squares_x: 8, squares_y: 8, square_mm: 36.0, marker_mm: 26.64 }
    }
}

impl From<BoardSpec> for BoardDims {
    fn from(b: BoardSpec) -> Self {
        BoardDims {
            squares_x: b.squares_x as u32,
            squares_y: b.squares_y as u32,
            square_mm: b.square_mm.into(),
            marker_mm: b.marker_mm.into(),
        }
    }
}

impl BoardDims {
    pub fn width_mm(&self) -> f64 {
        self.squares_x as f64 * self.square_mm
    }

    pub fn height_mm(&self) -> f64 {
        self.squares_y as f64 * self.square_mm
    }

    /// The inner ChArUco corners with OpenCV's ids (row-major, x fastest).
    pub fn corners(&self) -> impl Iterator<Item = (u32, V3)> + '_ {
        let nx = self.squares_x - 1;
        (0..nx * (self.squares_y - 1)).map(move |id| {
            let (i, j) = (id % nx + 1, id / nx + 1);
            (id, [i as f64 * self.square_mm, j as f64 * self.square_mm, 0.0])
        })
    }

    /// Reflectance of the printed face at board point `(x, y)`: black squares,
    /// white squares with a pseudo-random 7x7 marker bit pattern, and a grey
    /// table outside the print.
    pub fn albedo(&self, x: f64, y: f64) -> f64 {
        const BLACK: f64 = 0.08;
        const WHITE: f64 = 0.85;
        if x < 0.0 || y < 0.0 || x >= self.width_mm() || y >= self.height_mm() {
            return 0.4;
        }
        let (i, j) = ((x / self.square_mm) as u32, (y / self.square_mm) as u32);
        if (i + j) % 2 == 0 {
            return BLACK;
        }
        let (fx, fy) = (x - i as f64 * self.square_mm, y - j as f64 * self.square_mm);
        let m0 = 0.5 * (self.square_mm - self.marker_mm);
        let (mx, my) = (fx - m0, fy - m0);
        if mx < 0.0 || my < 0.0 || mx >= self.marker_mm || my >= self.marker_mm {
            return WHITE;
        }
        let cell = self.marker_mm / 7.0;
        let (ci, cj) = ((mx / cell) as u32, (my / cell) as u32);
        if ci == 0 || cj == 0 || ci == 6 || cj == 6 {
            return BLACK;
        }
        let h = (i * 7919 + j * 104_729 + ci * 31 + cj * 17).wrapping_mul(2_654_435_761);
        if (h >> 16) & 1 == 0 { BLACK } else { WHITE }
    }
}

/// Board plus solids.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Scene {
    pub board: BoardDims,
    pub shapes: Vec<Shape>,
}

/// Albedo of the solids (matte light grey).
pub const SHAPE_ALBEDO: f64 = 0.7;

impl Scene {
    /// Nearest hit of the ray `o + s·dir` (dir unit, board frame) with the
    /// board plane (camera side) or a solid.
    pub fn trace(&self, o: V3, dir: V3) -> Option<(f64, Surface)> {
        let mut best: Option<(f64, Surface)> = None;
        if dir[2].abs() > 1e-12 {
            let s = -o[2] / dir[2];
            if s > 1e-7 {
                best = Some((s, Surface::Board));
            }
        }
        for (i, sh) in self.shapes.iter().enumerate() {
            if let Some(s) = sh.hit(o, dir)
                && best.is_none_or(|(b, _)| s < b)
            {
                best = Some((s, Surface::Shape(i)));
            }
        }
        best
    }

    /// Whether the straight segment `a → b` reaches `b` unblocked (`b` on a surface).
    pub fn visible(&self, a: V3, b: V3) -> bool {
        let ab = sub(b, a);
        let len = norm(ab);
        if len < 1e-9 {
            return true;
        }
        let dir = scale(ab, 1.0 / len);
        self.trace(a, dir).is_none_or(|(s, _)| s >= len - 1e-4 * len.max(1.0))
    }

    pub fn albedo(&self, p: V3, surf: Surface) -> f64 {
        match surf {
            Surface::Board => self.board.albedo(p[0], p[1]),
            Surface::Shape(_) => SHAPE_ALBEDO,
        }
    }

    /// Signed distance of `p` to the nearest solid's surface, and that solid.
    pub fn nearest_shape(&self, p: V3) -> Option<(f64, usize)> {
        self.shapes.iter().enumerate().map(|(i, s)| (s.signed_distance(p), i)).min_by(|a, b| a.0.abs().total_cmp(&b.0.abs()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sphere_and_cuboid_hits() {
        let s = Shape::ball_on_board(0.0, 0.0, 10.0);
        let t = s.hit([0.0, 0.0, -100.0], [0.0, 0.0, 1.0]).unwrap();
        assert!((t - 80.0).abs() < 1e-9);
        assert!((s.signed_distance([0.0, 0.0, -30.0]) - 10.0).abs() < 1e-12);
        let c = Shape::block_on_board(0.0, 0.0, 10.0, 10.0, 5.0);
        let t = c.hit([5.0, 5.0, -100.0], [0.0, 0.0, 1.0]).unwrap();
        assert!((t - 95.0).abs() < 1e-9);
        assert!((c.signed_distance([5.0, 5.0, -7.0]) - 2.0).abs() < 1e-12);
        assert!((c.signed_distance([5.0, 5.0, -4.0]) + 1.0).abs() < 1e-12);
    }

    #[test]
    fn corners_follow_opencv_ids() {
        let b = BoardDims::default();
        let c: Vec<_> = b.corners().collect();
        assert_eq!(c.len(), 49);
        assert_eq!(c[0], (0, [36.0, 36.0, 0.0]));
        assert_eq!(c[1], (1, [72.0, 36.0, 0.0]));
        assert_eq!(c[7], (7, [36.0, 72.0, 0.0]));
    }

    #[test]
    fn occlusion() {
        let sc = Scene { board: BoardDims::default(), shapes: vec![Shape::ball_on_board(100.0, 100.0, 20.0)] };
        assert!(!sc.visible([100.0, 100.0, -300.0], [100.0, 100.0, 0.0]));
        assert!(sc.visible([100.0, 100.0, -300.0], [100.0, 100.0, -40.0]));
        assert!(sc.visible([0.0, 0.0, -300.0], [0.0, 0.0, 0.0]));
    }
}
