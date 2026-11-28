//! Live intrinsics self-calibration from the stream: collect steady, diverse
//! ChArUco views, then solve K on a background thread so the viewfinder
//! keeps going (`solve`).
//!
//! The calibration is a small state machine the app shows as it is:
//!
//! ```text
//!            start                 target + every side tilted
//!   Idle ──────────▶ Collecting ─────────────────────────────▶ Solving
//!    ▲  ▲             │    ▲                                  │      │
//!    │  │       pause │    │ start (resume)            gates ok│      │ a gate fails
//!    │  │             ▼    │                                  ▼      ▼
//!    │  │            Paused ◀── (leaving the mode)         Saved   Rejected
//!    │  └─ reset ────────────────────────────────────────────┘        │
//!    └──── reset ◀──────────────────────────────────────────────────── ┘
//!                    start: resume with the views kept, or begin anew when full
//! ```
//!
//! A rejected solve ends the collection: the reason stays on screen until
//! the user decides to add views (`start`) or begin anew (`reset`). Adding
//! views retries after [`LiveCalibrator::retry_step`] more of them; a set
//! that reaches [`LiveCalibrator::capacity`] is solved one last time, and a
//! rejection then asks for a fresh start.

mod solve;

use std::thread::JoinHandle;

use opencv::core::{Point2f, Vector};
use opencv::prelude::*;
use scan_algo::calib_coverage::{Coverage, CoverageCfg, SideSet, ViewSide};
use scan_algo::config::BoardSpec;
use scan_algo::geom::{Intrinsics, Pose};

pub use solve::{Calibration, Outcome, Rejection};
use solve::{Limits, ViewCorners};

/// Where the calibration is, as the app reads it from `h::CALIB_STATE`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalibState {
    Idle = 0,
    Collecting = 1,
    Solving = 2,
    Saved = 3,
    Rejected = 4,
    /// Collection stopped by the user or by another mode; the views are kept.
    Paused = 5,
}

/// What became of a view offered to the calibration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Capture {
    Added,
    /// Not collecting (idle, paused, solving or done).
    NotCollecting,
    TooFewCorners,
    /// Like a view already held: same corners, about the same place.
    Duplicate,
    /// The target is reached and the view adds no missing side.
    NotNeeded,
    /// The set holds [`LiveCalibrator::capacity`] views.
    Full,
}

/// How a `start` began.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Start {
    Fresh,
    /// Went on with the views already collected.
    Resumed,
}

struct View {
    corners: ViewCorners,
    centroid: (f32, f32),
}

pub struct LiveCalibrator {
    target: usize,
    coverage_cfg: CoverageCfg,
    min_corners: usize,
    state: CalibState,
    views: Vec<View>,
    coverage: Coverage,
    wh: Option<(u32, u32)>,
    /// The view count of the last solve; 0 before the first.
    solved_at: usize,
    job: Option<JoinHandle<Outcome>>,
    rejection: Option<Rejection>,
    last_rms: Option<f64>,
}

impl LiveCalibrator {
    pub fn new(target: usize) -> Self {
        let target = target.max(4);
        LiveCalibrator {
            target,
            coverage_cfg: CoverageCfg::for_target(target),
            min_corners: 8,
            state: CalibState::Idle,
            views: Vec::new(),
            coverage: Coverage::default(),
            wh: None,
            solved_at: 0,
            job: None,
            rejection: None,
            last_rms: None,
        }
    }

    /// A new target applies to the next set; a set being collected is
    /// started over, as its coverage was counted for the old one.
    pub fn set_target(&mut self, target: usize) {
        let target = target.max(4);
        if target != self.target {
            self.target = target;
            self.coverage_cfg = CoverageCfg::for_target(target);
            self.reset();
        }
    }

    pub fn state(&self) -> CalibState {
        self.state
    }

    /// Collecting views from the stream.
    pub fn collecting(&self) -> bool {
        self.state == CalibState::Collecting
    }

    pub fn solving(&self) -> bool {
        self.state == CalibState::Solving
    }

    pub fn count(&self) -> usize {
        self.views.len()
    }

    pub fn target(&self) -> usize {
        self.target
    }

    /// The most views a set holds.
    pub fn capacity(&self) -> usize {
        2 * self.target
    }

    /// New views a rejected set needs before it is solved again.
    pub fn retry_step(&self) -> usize {
        (self.target / 4).max(1)
    }

    /// The sides still short of tilted views.
    pub fn missing(&self) -> SideSet {
        self.coverage.missing(&self.coverage_cfg)
    }

    /// The side `pose` sees a board of `spec` from, as this calibration bins it.
    pub fn side(&self, pose: &Pose, spec: &BoardSpec) -> ViewSide {
        self.coverage_cfg.side(pose, [spec.width_mm() / 2.0, spec.height_mm() / 2.0, 0.0])
    }

    /// Why the last solve was not saved, while that is the state.
    pub fn rejection(&self) -> Option<&Rejection> {
        self.rejection.as_ref().filter(|_| self.state == CalibState::Rejected)
    }

    pub fn last_rms(&self) -> Option<f64> {
        self.last_rms
    }

    /// The views still to collect before the next solve can run (0 when
    /// only tilted views are missing).
    pub fn views_to_go(&self) -> usize {
        let next = if self.solved_at == 0 { self.target } else { (self.solved_at + self.retry_step()).min(self.capacity()) };
        next.saturating_sub(self.views.len())
    }

    /// A rejected set that cannot take more views: only a fresh start helps.
    pub fn exhausted(&self) -> bool {
        self.views.len() >= self.capacity()
    }

    /// Collect views: go on with a paused or rejected set that has room, or
    /// begin anew. Refused (None) while a solve runs.
    pub fn start(&mut self) -> Option<Start> {
        let resumable = matches!(self.state, CalibState::Paused | CalibState::Rejected) && !self.views.is_empty() && !self.exhausted();
        let how = match self.state {
            CalibState::Solving => return None,
            _ if resumable => Start::Resumed,
            _ => {
                self.clear_views();
                Start::Fresh
            }
        };
        self.state = CalibState::Collecting;
        self.rejection = None;
        Some(how)
    }

    /// Stop collecting and keep the views, for a later `start`. Only a
    /// collecting calibration pauses; every other state stays as it is.
    pub fn pause(&mut self) {
        if self.state == CalibState::Collecting {
            self.state = CalibState::Paused;
        }
    }

    /// Back to idle with no views. A solve still running is abandoned: its
    /// views belong to the set being dropped (another camera, focus or
    /// board), so its result must never be saved. The thread finishes on
    /// its own.
    pub fn reset(&mut self) {
        self.job = None;
        self.clear_views();
        self.state = CalibState::Idle;
        self.rejection = None;
        self.last_rms = None;
    }

    fn clear_views(&mut self) {
        self.views.clear();
        self.coverage.clear();
        self.solved_at = 0;
        self.wh = None;
    }

    /// Offer one detected view, seen from `side`. Refuses too few corners
    /// and near-duplicates (same corner count, centroid moved less than
    /// ~8 px at 1920 px), so holding the board still does not flood the
    /// set. Past the target, a first set takes only views of sides still
    /// missing. Views must share one resolution; a different one starts
    /// the set over.
    pub fn offer(&mut self, corners: &Vector<Point2f>, ids: &Vector<i32>, wh: (u32, u32), side: ViewSide) -> Capture {
        if self.state != CalibState::Collecting {
            return Capture::NotCollecting;
        }
        if corners.len() < self.min_corners {
            return Capture::TooFewCorners;
        }
        if self.wh != Some(wh) {
            self.clear_views();
            self.wh = Some(wh);
        }
        if self.exhausted() {
            return Capture::Full;
        }
        if self.solved_at == 0 && self.views.len() >= self.target && !self.coverage.wants(&self.coverage_cfg, side) {
            return Capture::NotNeeded;
        }
        let corners = corners.to_vec();
        let centroid = mean(&corners);
        let tol = 8.0 * (wh.0.max(wh.1) as f32 / 1920.0).max(0.25);
        let dup = self
            .views
            .iter()
            .any(|v| v.corners.corners.len() == corners.len() && (v.centroid.0 - centroid.0).hypot(v.centroid.1 - centroid.1) < tol);
        if dup {
            return Capture::Duplicate;
        }
        self.views.push(View { corners: ViewCorners { corners, ids: ids.to_vec() }, centroid });
        self.coverage.add(side);
        Capture::Added
    }

    /// Start the background solve once the set is ready: the target
    /// reached with every side tilted, or, after a rejection,
    /// [`Self::retry_step`] more views (or a full set). With a
    /// `surface_degree` the board's shape is measured along.
    pub fn maybe_solve(&mut self, spec: BoardSpec, seed: Intrinsics, surface_degree: Option<u8>) -> bool {
        let Some(wh) = self.wh else { return false };
        if self.state != CalibState::Collecting || self.views.len() < self.target || (!self.missing().is_empty() && !self.exhausted()) {
            return false;
        }
        if self.solved_at > 0 && self.views_to_go() > 0 {
            return false;
        }
        self.solved_at = self.views.len();
        self.state = CalibState::Solving;
        let views: Vec<ViewCorners> =
            self.views.iter().map(|v| ViewCorners { corners: v.corners.corners.clone(), ids: v.corners.ids.clone() }).collect();
        let limits = Limits { min_corners: self.min_corners, min_views: self.target, surface_degree };
        self.job = Some(std::thread::spawn(move || {
            solve::solve(spec, &views, wh, seed, limits).unwrap_or_else(|e| Outcome::Rejected(Rejection::SolverFailed(e.to_string())))
        }));
        true
    }

    /// The finished background solve, if any: a saved one ends the set, a
    /// rejected one stops the collection with its reason.
    pub fn poll(&mut self) -> Option<Outcome> {
        if !self.job.as_ref()?.is_finished() {
            return None;
        }
        let out = self.job.take()?.join().unwrap_or_else(|_| Outcome::Rejected(Rejection::SolverFailed("solver panicked".into())));
        match &out {
            Outcome::Solved(c) => {
                self.last_rms = Some(c.rms);
                self.clear_views();
                self.state = CalibState::Saved;
            }
            Outcome::Rejected(r) => {
                self.last_rms = r.rms();
                self.rejection = Some(r.clone());
                self.state = CalibState::Rejected;
            }
        }
        Some(out)
    }
}

fn mean(c: &[Point2f]) -> (f32, f32) {
    let n = c.len().max(1) as f32;
    (c.iter().map(|p| p.x).sum::<f32>() / n, c.iter().map(|p| p.y).sum::<f32>() / n)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A view of `n` corners around `(x, y)`.
    fn view(x: f32, y: f32) -> (Vector<Point2f>, Vector<i32>) {
        let corners: Vector<Point2f> = (0..10).map(|i| Point2f::new(x + i as f32, y)).collect();
        let ids: Vector<i32> = (0..10).collect();
        (corners, ids)
    }

    fn seed() -> Intrinsics {
        Intrinsics { fx: 1500.0, fy: 1500.0, cx: 960.0, cy: 540.0, dist: [0.0; 5], width: 1920.0, height: 1080.0 }
    }

    fn offer(c: &mut LiveCalibrator, i: usize, side: ViewSide) -> Capture {
        let (corners, ids) = view(40.0 * i as f32, 0.0);
        c.offer(&corners, &ids, (1920, 1080), side)
    }

    #[test]
    fn views_are_taken_only_while_collecting() {
        let mut c = LiveCalibrator::new(16);
        assert_eq!(offer(&mut c, 0, ViewSide::Frontal), Capture::NotCollecting);
        assert_eq!(c.start(), Some(Start::Fresh));
        assert_eq!(offer(&mut c, 0, ViewSide::Frontal), Capture::Added);
        assert_eq!(offer(&mut c, 0, ViewSide::Frontal), Capture::Duplicate);
        c.pause();
        assert_eq!(c.state(), CalibState::Paused);
        assert_eq!(offer(&mut c, 1, ViewSide::Frontal), Capture::NotCollecting);
    }

    #[test]
    fn pause_and_start_resume_the_set() {
        let mut c = LiveCalibrator::new(16);
        c.start();
        for i in 0..5 {
            offer(&mut c, i, ViewSide::Left);
        }
        c.pause();
        assert_eq!(c.start(), Some(Start::Resumed));
        assert_eq!(c.count(), 5);
        c.reset();
        assert_eq!(c.state(), CalibState::Idle);
        assert_eq!(c.start(), Some(Start::Fresh));
        assert_eq!(c.count(), 0);
    }

    #[test]
    fn past_the_target_only_missing_sides_are_taken() {
        let mut c = LiveCalibrator::new(16);
        c.start();
        for i in 0..16 {
            assert_eq!(offer(&mut c, i, ViewSide::Frontal), Capture::Added);
        }
        assert_eq!(c.missing().bits(), 0b1111);
        // no solve yet: the views do not pin the focal length down
        assert!(!c.maybe_solve(BoardSpec::default(), seed(), None));
        assert_eq!(offer(&mut c, 16, ViewSide::Frontal), Capture::NotNeeded);
        for (i, side) in (17..).zip(ViewSide::TILTED) {
            assert_eq!(offer(&mut c, i, side), Capture::Added);
        }
        assert!(c.missing().is_empty());
        assert_eq!(c.count(), 20);
        assert_eq!(c.views_to_go(), 0);
    }

    #[test]
    fn a_new_resolution_starts_the_set_over() {
        let mut c = LiveCalibrator::new(16);
        c.start();
        offer(&mut c, 0, ViewSide::Left);
        let (corners, ids) = view(500.0, 0.0);
        assert_eq!(c.offer(&corners, &ids, (1280, 720), ViewSide::Left), Capture::Added);
        assert_eq!(c.count(), 1);
    }

    #[test]
    fn a_rejection_is_kept_until_the_user_decides() {
        let mut c = LiveCalibrator::new(16);
        c.start();
        for i in 0..16 {
            offer(&mut c, i, ViewSide::TILTED[i % 4]);
        }
        // stand in for a finished solve that a gate refused
        c.solved_at = c.count();
        c.state = CalibState::Rejected;
        c.rejection = Some(Rejection::FocalUncertain { rms: 0.4, sigma: 5.0, f: 1500.0 });
        assert!(c.rejection().is_some());
        assert_eq!(offer(&mut c, 20, ViewSide::Left), Capture::NotCollecting);
        // adding views: the set is kept, the next solve waits for a retry step
        assert_eq!(c.start(), Some(Start::Resumed));
        assert!(c.rejection().is_none());
        assert_eq!(c.views_to_go(), c.retry_step());
        assert!(!c.maybe_solve(BoardSpec::default(), seed(), None));
        // a rejected set that is full only starts anew
        for i in 0..40 {
            offer(&mut c, 100 + i, ViewSide::Frontal);
        }
        assert_eq!(c.count(), c.capacity());
        assert_eq!(offer(&mut c, 500, ViewSide::Left), Capture::Full);
        c.state = CalibState::Rejected;
        assert_eq!(c.start(), Some(Start::Fresh));
        assert_eq!(c.count(), 0);
    }
}
