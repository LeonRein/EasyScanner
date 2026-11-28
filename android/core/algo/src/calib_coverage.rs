//! Which viewing angles a camera calibration's views cover.
//!
//! Views of a plane seen square-on cannot tell the focal length from the
//! distance: shrinking f and moving closer give the same picture. Only views
//! with foreshortening, the board seen from the side, pin f down. A set of
//! views collected while the phone moves parallel to the board is therefore
//! diverse by position but useless for K, and the solve's focal-uncertainty
//! gate rejects it however many such views it holds.
//!
//! So every view is binned by the side of the board's normal the camera
//! looks from, and the calibration waits until each side holds a few views
//! tilted at least [`CoverageCfg::min_tilt_deg`]. Sides are named in the
//! camera image (x right, y down): "left" means the camera looks at the
//! board from the left of its normal.

use crate::geom::{Pose, V3, mat_vec, norm, sub};

/// Where the camera looks at the board from, relative to its normal.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewSide {
    /// Within [`CoverageCfg::min_tilt_deg`] of square-on.
    Frontal,
    Left,
    Right,
    Above,
    Below,
}

impl ViewSide {
    /// The tilted sides, in the order of [`Coverage::missing`]'s bits.
    pub const TILTED: [ViewSide; 4] = [ViewSide::Left, ViewSide::Right, ViewSide::Above, ViewSide::Below];

    /// This side's bit in a [`SideSet`]; none for [`ViewSide::Frontal`].
    fn bit(self) -> Option<u8> {
        Self::TILTED.iter().position(|&s| s == self).map(|i| 1 << i)
    }
}

impl std::fmt::Display for ViewSide {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            ViewSide::Frontal => "square-on",
            ViewSide::Left => "the left",
            ViewSide::Right => "the right",
            ViewSide::Above => "above",
            ViewSide::Below => "below",
        })
    }
}

/// A set of tilted [`ViewSide`]s as a bit mask: bit 0 left, 1 right,
/// 2 above, 3 below. The app reads it from a header slot.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SideSet(u8);

impl SideSet {
    pub fn bits(self) -> u8 {
        self.0
    }

    pub fn is_empty(self) -> bool {
        self.0 == 0
    }

    pub fn contains(self, side: ViewSide) -> bool {
        side.bit().is_some_and(|b| self.0 & b != 0)
    }

    pub fn sides(self) -> impl Iterator<Item = ViewSide> {
        ViewSide::TILTED.into_iter().filter(move |&s| self.contains(s))
    }
}

/// How many tilted views a calibration needs, and how tilted.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoverageCfg {
    /// A view is tilted when the line of sight is at least this far from
    /// the board's normal (degrees).
    pub min_tilt_deg: f64,
    /// Tilted views needed on each of the four sides.
    pub per_side: usize,
}

impl CoverageCfg {
    /// The requirement for a calibration of `target` views: a sixteenth of
    /// them on each side, so a quarter of the set is tilted.
    pub fn for_target(target: usize) -> Self {
        CoverageCfg { min_tilt_deg: 20.0, per_side: (target / 16).max(1) }
    }

    /// The side `pose` sees the board from: the way the camera centre is
    /// displaced from the line through the board's `centre` (board frame,
    /// mm) along its normal, in image directions.
    pub fn side(&self, pose: &Pose, centre: V3) -> ViewSide {
        let c = sub(pose.camera_position(), centre);
        let tilt = (c[2].abs() / norm(c).max(f64::MIN_POSITIVE)).clamp(0.0, 1.0).acos().to_degrees();
        if tilt < self.min_tilt_deg {
            return ViewSide::Frontal;
        }
        let off = mat_vec(&pose.r, [c[0], c[1], 0.0]);
        match (off[0].abs() >= off[1].abs(), off[0] < 0.0, off[1] < 0.0) {
            (true, true, _) => ViewSide::Left,
            (true, false, _) => ViewSide::Right,
            (false, _, true) => ViewSide::Above,
            (false, _, false) => ViewSide::Below,
        }
    }
}

/// The tilted views collected per side.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Coverage {
    counts: [usize; 4],
}

impl Coverage {
    pub fn add(&mut self, side: ViewSide) {
        if let Some(i) = ViewSide::TILTED.iter().position(|&s| s == side) {
            self.counts[i] += 1;
        }
    }

    pub fn clear(&mut self) {
        self.counts = [0; 4];
    }

    /// The sides with fewer than `cfg.per_side` views.
    pub fn missing(&self, cfg: &CoverageCfg) -> SideSet {
        let bits = self.counts.iter().enumerate().filter(|&(_, &n)| n < cfg.per_side).fold(0, |m, (i, _)| m | 1 << i);
        SideSet(bits)
    }

    /// Whether a view from `side` adds to a side still short of views.
    pub fn wants(&self, cfg: &CoverageCfg, side: ViewSide) -> bool {
        self.missing(cfg).contains(side)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::{mat_mul, rodrigues, scale};

    /// The camera at `c` (board frame, mm; the board's +Z points away from
    /// the viewer, so c[2] < 0) looking at the board centre, its image x
    /// axis along the board's x as far as the view allows.
    fn looking_from(c: [f64; 3]) -> Pose {
        let z = scale(c, -1.0 / norm(c));
        let x0 = [1.0, 0.0, 0.0];
        let x = {
            let d = x0[0] * z[0];
            let v = [x0[0] - d * z[0], -d * z[1], -d * z[2]];
            scale(v, 1.0 / norm(v))
        };
        let y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
        // rows: the camera axes in board coordinates
        let r = [x, y, z];
        let t = scale(mat_vec(&r, c), -1.0);
        Pose { r, t }
    }

    #[test]
    fn square_on_is_frontal() {
        let cfg = CoverageCfg::for_target(64);
        assert_eq!(cfg.side(&looking_from([0.0, 0.0, -400.0]), [0.0; 3]), ViewSide::Frontal);
        // 10° off the normal
        assert_eq!(cfg.side(&looking_from([70.0, 0.0, -400.0]), [0.0; 3]), ViewSide::Frontal);
    }

    #[test]
    fn tilted_views_are_named_after_where_the_camera_is() {
        let cfg = CoverageCfg::for_target(64);
        // 30° off the normal to each side; the image x runs along the board's x,
        // and the image y (down) along the board's y
        let d = 400.0 * 30f64.to_radians().tan();
        assert_eq!(cfg.side(&looking_from([-d, 0.0, -400.0]), [0.0; 3]), ViewSide::Left);
        assert_eq!(cfg.side(&looking_from([d, 0.0, -400.0]), [0.0; 3]), ViewSide::Right);
        assert_eq!(cfg.side(&looking_from([0.0, -d, -400.0]), [0.0; 3]), ViewSide::Above);
        assert_eq!(cfg.side(&looking_from([0.0, d, -400.0]), [0.0; 3]), ViewSide::Below);
    }

    #[test]
    fn the_side_follows_the_image_not_the_board() {
        // the phone turned by 90° about its axis: the board's x is now the
        // image's y, so a camera off along the board's x is above or below
        let cfg = CoverageCfg::for_target(64);
        let d = 400.0 * 30f64.to_radians().tan();
        let p = looking_from([d, 0.0, -400.0]);
        let quarter = rodrigues([0.0, 0.0, std::f64::consts::FRAC_PI_2]);
        let turned = Pose { r: mat_mul(&quarter, &p.r), t: mat_vec(&quarter, p.t) };
        assert!(matches!(cfg.side(&turned, [0.0; 3]), ViewSide::Above | ViewSide::Below));
    }

    #[test]
    fn the_board_off_centre_but_square_on_is_frontal() {
        // the board parallel to the sensor at the image edge
        let cfg = CoverageCfg::for_target(64);
        let pose = Pose { r: rodrigues([0.0, 0.0, 0.0]), t: [120.0, 0.0, 400.0] };
        // 16.7° off the normal: below the tilt threshold
        assert_eq!(cfg.side(&pose, [0.0; 3]), ViewSide::Frontal);
    }

    #[test]
    fn coverage_counts_until_every_side_has_its_views() {
        let cfg = CoverageCfg { min_tilt_deg: 20.0, per_side: 2 };
        let mut c = Coverage::default();
        assert_eq!(c.missing(&cfg).bits(), 0b1111);
        c.add(ViewSide::Frontal);
        c.add(ViewSide::Left);
        c.add(ViewSide::Left);
        c.add(ViewSide::Above);
        let missing = c.missing(&cfg);
        assert!(!missing.contains(ViewSide::Left));
        assert!(missing.contains(ViewSide::Above));
        assert_eq!(missing.sides().collect::<Vec<_>>(), vec![ViewSide::Right, ViewSide::Above, ViewSide::Below]);
        assert!(c.wants(&cfg, ViewSide::Right));
        assert!(!c.wants(&cfg, ViewSide::Left));
        assert!(!c.wants(&cfg, ViewSide::Frontal));
        for s in [ViewSide::Right, ViewSide::Right, ViewSide::Above, ViewSide::Below, ViewSide::Below] {
            c.add(s);
        }
        assert!(c.missing(&cfg).is_empty());
        c.clear();
        assert_eq!(c.missing(&cfg).bits(), 0b1111);
    }
}
