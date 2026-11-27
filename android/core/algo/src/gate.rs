//! Quality gates: which frames may add points to the scan (and lines to the
//! laser-plane calibration). A laser point is only as good as the board pose
//! it is triangulated with, so a frame whose pose is weak — a poor fit, few
//! corners, an ambiguous planar solution, or a camera moving fast enough to
//! blur and shear the image — is shown, but not kept.

use serde::Deserialize;

/// Thresholds for the scan's per-frame gate. Every check can be switched off
/// on its own: 0 (or a non-finite value) disables it.
#[derive(Deserialize, Clone, Copy, Debug, PartialEq)]
#[serde(default)]
pub struct ScanGateCfg {
    /// Off: every posed frame adds points, as before the gate existed.
    pub enabled: bool,
    /// Board pose reprojection RMS above this (px) rejects the frame.
    pub max_reproj_px: f64,
    /// Fewer ChArUco corners than this rejects the frame.
    pub min_corners: usize,
    /// Runner-up IPPE branch error / chosen branch error below this rejects
    /// the frame: near-frontal views where the two planar solutions tie and
    /// the tilt can flip. A pose from the iterative fallback (no margin) is
    /// rejected too while this check is on.
    pub min_ippe_margin: f64,
    /// Camera moving faster than this (mm/s) rejects the frame: motion blur,
    /// and the rolling shutter shears the stripe against the board.
    pub max_speed_mm_s: f64,
    /// … or turning faster than this (deg/s).
    pub max_turn_deg_s: f64,
    /// Without a speed estimate (the board was just (re)acquired) the frame
    /// is rejected rather than trusted; only while a speed limit is set.
    pub require_speed: bool,
}

impl Default for ScanGateCfg {
    fn default() -> Self {
        ScanGateCfg {
            enabled: true,
            // synthetic views fit at < 0.5 px; a real phone at 1080p well
            // under 1 px — 1.5 lets through all but the clearly broken
            max_reproj_px: 1.5,
            // a third of the 8x8 board's 49 inner corners
            min_corners: 16,
            // board.rs already treats < 3 as ambiguous and disambiguates
            // against the previous pose; below 1.5 that guess is weak
            min_ippe_margin: 1.5,
            // a slow hand sweep is ~20–60 mm/s; at 120 mm/s and 30 fps the
            // camera moves 4 mm per frame
            max_speed_mm_s: 120.0,
            max_turn_deg_s: 25.0,
            require_speed: true,
        }
    }
}

/// What one frame offers the gate.
#[derive(Clone, Copy, Debug)]
pub struct FrameQuality {
    pub reproj_rms: f64,
    pub corners: usize,
    /// NaN when IPPE gave no alternative branch (iterative fallback).
    pub margin: f64,
    /// Camera speed (mm/s, deg/s) over the recent window, if known.
    pub speed: Option<(f64, f64)>,
}

/// Why a frame was not kept. The numeric codes go into the frame header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reject {
    PoseRms = 1,
    FewCorners = 2,
    Ambiguous = 3,
    Moving = 4,
    Settling = 5,
}

fn on(limit: f64) -> bool {
    limit.is_finite() && limit > 0.0
}

/// `x ≤ limit`, false for NaN: an unknown value never passes.
fn at_most(x: f64, limit: f64) -> bool {
    x <= limit
}

impl ScanGateCfg {
    /// The pose-only checks (no motion): fit, corners, IPPE margin. Used for
    /// the laser-plane calibration, where the board is tilted by hand
    /// between still views and the motion check would get in the way.
    pub fn check_pose(&self, q: &FrameQuality) -> Result<(), Reject> {
        if !self.enabled {
            return Ok(());
        }
        if on(self.max_reproj_px) && !at_most(q.reproj_rms, self.max_reproj_px) {
            return Err(Reject::PoseRms);
        }
        if self.min_corners > 0 && q.corners < self.min_corners {
            return Err(Reject::FewCorners);
        }
        if on(self.min_ippe_margin) && !at_most(self.min_ippe_margin, q.margin) {
            return Err(Reject::Ambiguous);
        }
        Ok(())
    }

    /// Every check: the pose, then the motion.
    pub fn check(&self, q: &FrameQuality) -> Result<(), Reject> {
        self.check_pose(q)?;
        if !self.enabled {
            return Ok(());
        }
        let limits = on(self.max_speed_mm_s) || on(self.max_turn_deg_s);
        if !limits {
            return Ok(());
        }
        match q.speed {
            None if self.require_speed => Err(Reject::Settling),
            None => Ok(()),
            Some((v, w)) => {
                if (on(self.max_speed_mm_s) && !at_most(v, self.max_speed_mm_s))
                    || (on(self.max_turn_deg_s) && !at_most(w, self.max_turn_deg_s))
                {
                    Err(Reject::Moving)
                } else {
                    Ok(())
                }
            }
        }
    }
}

/// Running tally of the gate's verdicts during a scan.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GateStats {
    pub kept: u64,
    pub pose_rms: u64,
    pub few_corners: u64,
    pub ambiguous: u64,
    pub moving: u64,
    pub settling: u64,
}

impl GateStats {
    pub fn record(&mut self, r: Result<(), Reject>) {
        match r {
            Ok(()) => self.kept += 1,
            Err(Reject::PoseRms) => self.pose_rms += 1,
            Err(Reject::FewCorners) => self.few_corners += 1,
            Err(Reject::Ambiguous) => self.ambiguous += 1,
            Err(Reject::Moving) => self.moving += 1,
            Err(Reject::Settling) => self.settling += 1,
        }
    }

    pub fn rejected(&self) -> u64 {
        self.pose_rms + self.few_corners + self.ambiguous + self.moving + self.settling
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn good() -> FrameQuality {
        FrameQuality { reproj_rms: 0.4, corners: 40, margin: 12.0, speed: Some((30.0, 5.0)) }
    }

    #[test]
    fn a_good_frame_passes() {
        assert_eq!(ScanGateCfg::default().check(&good()), Ok(()));
    }

    #[test]
    fn each_check_rejects_with_its_reason() {
        let g = ScanGateCfg::default();
        assert_eq!(g.check(&FrameQuality { reproj_rms: 2.0, ..good() }), Err(Reject::PoseRms));
        assert_eq!(g.check(&FrameQuality { reproj_rms: f64::NAN, ..good() }), Err(Reject::PoseRms));
        assert_eq!(g.check(&FrameQuality { corners: 10, ..good() }), Err(Reject::FewCorners));
        assert_eq!(g.check(&FrameQuality { margin: 1.1, ..good() }), Err(Reject::Ambiguous));
        assert_eq!(g.check(&FrameQuality { margin: f64::NAN, ..good() }), Err(Reject::Ambiguous), "iterative fallback");
        assert_eq!(g.check(&FrameQuality { speed: Some((300.0, 5.0)), ..good() }), Err(Reject::Moving));
        assert_eq!(g.check(&FrameQuality { speed: Some((30.0, 60.0)), ..good() }), Err(Reject::Moving));
        assert_eq!(g.check(&FrameQuality { speed: None, ..good() }), Err(Reject::Settling));
    }

    #[test]
    fn pose_check_ignores_motion() {
        let g = ScanGateCfg::default();
        let fast = FrameQuality { speed: Some((900.0, 200.0)), ..good() };
        assert_eq!(g.check_pose(&fast), Ok(()));
        assert_eq!(g.check_pose(&FrameQuality { speed: None, ..good() }), Ok(()));
        assert_eq!(g.check_pose(&FrameQuality { corners: 4, ..fast }), Err(Reject::FewCorners));
    }

    #[test]
    fn checks_switch_off_one_by_one() {
        let bad = FrameQuality { reproj_rms: 9.0, corners: 4, margin: 1.0, speed: None };
        let all_off = ScanGateCfg {
            max_reproj_px: 0.0,
            min_corners: 0,
            min_ippe_margin: 0.0,
            max_speed_mm_s: 0.0,
            max_turn_deg_s: 0.0,
            ..ScanGateCfg::default()
        };
        assert_eq!(all_off.check(&bad), Ok(()));
        assert_eq!(ScanGateCfg { enabled: false, ..ScanGateCfg::default() }.check(&bad), Ok(()));
        let no_require = ScanGateCfg { require_speed: false, ..ScanGateCfg::default() };
        assert_eq!(no_require.check(&FrameQuality { speed: None, ..good() }), Ok(()));
    }

    #[test]
    fn config_json_is_partial() {
        let g: ScanGateCfg = serde_json::from_str(r#"{"max_reproj_px":0.8}"#).unwrap();
        assert_eq!(g.max_reproj_px, 0.8);
        assert_eq!(g.min_corners, ScanGateCfg::default().min_corners);
    }

    #[test]
    fn stats_tally() {
        let g = ScanGateCfg::default();
        let mut s = GateStats::default();
        s.record(g.check(&good()));
        s.record(g.check(&FrameQuality { corners: 3, ..good() }));
        s.record(g.check(&FrameQuality { speed: Some((500.0, 0.0)), ..good() }));
        assert_eq!((s.kept, s.few_corners, s.moving, s.rejected()), (1, 1, 1, 2));
    }
}
