//! Keeping the board's measured shape current without asking the user.
//!
//! The shape ([`BoardSurface`]) belongs to one placement of the board: it
//! holds for every scan while the board lies where it is, and is wrong once
//! the board is laid down anew. There is no check to run at app start.
//! Instead every steady, well-posed frame offers its corners
//! ([`SurfaceMonitor::offer`]); once they cover enough directions the
//! monitor hands out a fit of the recent views ([`SurfaceMonitor::job`],
//! run off the frame thread), and [`SurfaceMonitor::accept`] compares its
//! result with the shape in use:
//!
//! * no shape yet: a well-determined fit becomes the shape;
//! * the same within the fits' uncertainty: confirmed;
//! * clearly different: suspect. The views are dropped and only views taken
//!   from now on decide: a second fit that differs too replaces the shape
//!   (the board was moved), one that agrees confirms it (the first was a
//!   fluke).
//!
//! The user does nothing but move the phone around the board, which every
//! calibration and scan does anyway.

use serde::{Deserialize, Serialize};

use crate::board_surface::{BoardSurface, SurfaceFit, SurfaceView, SurfaceViews, fit};
use crate::geom::Intrinsics;

/// How the board's shape is known, for the app's status line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SurfaceState {
    /// Nothing measured yet: the board is taken as flat.
    Unmeasured = 0,
    /// Collecting views for the first measurement, or for a re-measurement
    /// after a fit disagreed with the shape in use.
    Measuring = 1,
    /// The shape in use agrees with the latest views.
    Verified = 2,
    /// The latest views showed another shape and replaced the one in use:
    /// the board was laid down anew.
    Replaced = 3,
}

/// A measured shape and how well it was determined.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MeasuredSurface {
    pub surface: BoardSurface,
    /// Reprojection RMS of the fit with this shape, and with a flat board (px).
    pub rms_px: f64,
    pub flat_rms_px: f64,
    /// Largest standard deviation of the height over the board (mm).
    pub height_sigma_mm: f64,
}

impl From<SurfaceFit> for MeasuredSurface {
    fn from(f: SurfaceFit) -> Self {
        MeasuredSurface { surface: f.surface, rms_px: f.rms_px, flat_rms_px: f.flat_rms_px, height_sigma_mm: f.height_sigma_mm }
    }
}

/// What [`SurfaceMonitor::accept`] made of a fit.
#[derive(Clone, Debug, PartialEq)]
pub enum Verdict {
    /// The fit failed, or does not pin the shape down well enough yet.
    Undetermined,
    /// The views do not fit one shape (taken from two placements, or the
    /// corners are poor): they were dropped.
    Inconsistent { rms_px: f64 },
    /// The first measured shape: now in use.
    First,
    /// The shape in use agrees with the fit; `refined` when the fit, better
    /// determined, took its place.
    Confirmed { difference_mm: f64, refined: bool },
    /// The fit differs from the shape in use: re-measuring from fresh views.
    Suspect { difference_mm: f64 },
    /// Fresh views confirmed a different shape: it replaced the one in use.
    Replaced { difference_mm: f64 },
}

impl Verdict {
    /// The shape in use changed and should be saved.
    pub fn changed_shape(&self) -> bool {
        matches!(self, Verdict::First | Verdict::Replaced { .. } | Verdict::Confirmed { refined: true, .. })
    }
}

/// A shape fit to run off the frame thread.
#[derive(Clone, Debug)]
pub struct FitJob {
    views: Vec<SurfaceView>,
    start: BoardSurface,
    width_mm: f64,
    height_mm: f64,
}

impl FitJob {
    pub fn run(&self, k: &Intrinsics) -> Option<SurfaceFit> {
        fit(k, &self.start, &self.views, self.width_mm, self.height_mm)
    }
}

/// See the [module documentation](self).
#[derive(Clone, Debug)]
pub struct SurfaceMonitor {
    width_mm: f64,
    height_mm: f64,
    degree: u8,
    views: SurfaceViews,
    current: Option<MeasuredSurface>,
    flat: BoardSurface,
    state: SurfaceState,
    /// A fit disagreed with the shape in use; the next one decides.
    suspect: bool,
    last_job_s: Option<f64>,
}

impl SurfaceMonitor {
    /// Seconds between fits.
    pub const REFIT_S: f64 = 2.0;
    /// A fit must pin the height down to this (mm, one sigma) to be used.
    pub const MAX_SIGMA_MM: f64 = 0.15;
    /// Views fitting one shape worse than this (px RMS) do not show one
    /// placement of the board.
    pub const MAX_RMS_PX: f64 = 1.0;
    /// Two shapes closer than this (mm) are the same placement, whatever
    /// their uncertainty.
    pub const MIN_CHANGE_MM: f64 = 0.25;

    /// For a `width`×`height` mm board, with height terms up to `degree`.
    pub fn new(width_mm: f64, height_mm: f64, degree: u8) -> Self {
        SurfaceMonitor {
            width_mm,
            height_mm,
            degree,
            views: SurfaceViews::new(width_mm, height_mm),
            current: None,
            flat: BoardSurface::flat(),
            state: SurfaceState::Unmeasured,
            suspect: false,
            last_job_s: None,
        }
    }

    /// Start over without a shape (another board).
    pub fn reset(&mut self) {
        *self = SurfaceMonitor::new(self.width_mm, self.height_mm, self.degree);
    }

    /// Use a shape measured before (the saved one, or one the camera
    /// calibration measured). It is verified like any other: a board moved
    /// since is noticed by the next fits.
    pub fn restore(&mut self, measured: MeasuredSurface) {
        self.current = Some(measured);
        self.state = SurfaceState::Measuring;
        self.suspect = false;
    }

    /// The shape poses and stripes are computed against: the measured one,
    /// else the plane z = 0.
    pub fn surface(&self) -> &BoardSurface {
        self.current.as_ref().map_or(&self.flat, |m| &m.surface)
    }

    pub fn measured(&self) -> Option<&MeasuredSurface> {
        self.current.as_ref()
    }

    pub fn state(&self) -> SurfaceState {
        self.state
    }

    /// Views collected toward the next fit, and how many it needs at least.
    pub fn progress(&self) -> (usize, usize) {
        (self.views.len(), SurfaceViews::MIN_VIEWS)
    }

    /// Drop the views collected so far (taken through another camera model
    /// or at another resolution); the shape in use stays.
    pub fn forget_views(&mut self) {
        self.views.clear();
        self.last_job_s = None;
    }

    /// Offer a steady frame's corners (see [`SurfaceViews::offer`]).
    pub fn offer(&mut self, t_s: f64, view: SurfaceView) -> bool {
        if self.current.is_none() && self.state == SurfaceState::Unmeasured {
            self.state = SurfaceState::Measuring;
        }
        self.views.offer(t_s, view)
    }

    /// A fit of the views collected, at most every [`Self::REFIT_S`] and
    /// only once they cover enough directions. The caller runs it (off the
    /// frame thread) and hands the result to [`Self::accept`].
    pub fn job(&mut self, t_s: f64) -> Option<FitJob> {
        if !self.views.ready() || self.last_job_s.is_some_and(|t| (0.0..Self::REFIT_S).contains(&(t_s - t))) {
            return None;
        }
        self.last_job_s = Some(t_s);
        let start = match &self.current {
            Some(m) if m.surface.degree() == self.degree => m.surface.clone(),
            _ => BoardSurface::for_board(self.width_mm, self.height_mm, self.degree),
        };
        Some(FitJob { views: self.views.views(), start, width_mm: self.width_mm, height_mm: self.height_mm })
    }

    /// Weigh a finished fit (`None` when it failed) against the shape in use.
    pub fn accept(&mut self, result: Option<SurfaceFit>) -> Verdict {
        let Some(new) = result.filter(|f| f.height_sigma_mm.is_finite() && f.height_sigma_mm <= Self::MAX_SIGMA_MM) else {
            return Verdict::Undetermined;
        };
        if new.rms_px.is_nan() || new.rms_px > Self::MAX_RMS_PX {
            self.views.clear();
            self.state = SurfaceState::Measuring;
            return Verdict::Inconsistent { rms_px: new.rms_px };
        }
        let Some(old) = &self.current else {
            self.current = Some(new.into());
            self.state = SurfaceState::Verified;
            return Verdict::First;
        };
        let difference_mm = old.surface.max_difference(&new.surface, self.width_mm, self.height_mm);
        let tolerance = Self::MIN_CHANGE_MM.max(3.0 * old.height_sigma_mm.hypot(new.height_sigma_mm));
        if difference_mm <= tolerance {
            self.suspect = false;
            self.state = SurfaceState::Verified;
            // a better-determined measurement of the same shape refines it
            let refined = new.height_sigma_mm < 0.5 * old.height_sigma_mm;
            if refined {
                self.current = Some(new.into());
            }
            return Verdict::Confirmed { difference_mm, refined };
        }
        if !self.suspect {
            // one fit could be thrown by a few poor views: re-measure from
            // views taken from now on before replacing anything
            self.suspect = true;
            self.views.clear();
            self.state = SurfaceState::Measuring;
            return Verdict::Suspect { difference_mm };
        }
        self.suspect = false;
        self.current = Some(new.into());
        self.state = SurfaceState::Replaced;
        Verdict::Replaced { difference_mm }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board_surface::tests::{H, W, observe, view_pose, warped};
    use crate::plane::Rng;

    fn feed(m: &mut SurfaceMonitor, truth: &BoardSurface, t0: f64, rng: &mut Rng) -> Option<FitJob> {
        let mut t = t0;
        for (i, el) in [45.0, 55.0, 65.0, 75.0].into_iter().enumerate() {
            for az in (0..6).map(|a| 60.0 * a as f64 + 15.0 * i as f64) {
                m.offer(t, observe(truth, &view_pose(el, az, 420.0), 0.15, rng));
                t += 0.1;
            }
        }
        m.job(t)
    }

    fn k() -> Intrinsics {
        crate::board_surface::tests::k()
    }

    #[test]
    fn a_shape_is_measured_kept_and_replaced_when_the_board_moves() {
        let mut rng = Rng::new(11);
        let mut m = SurfaceMonitor::new(W, H, 3);
        assert_eq!(m.state(), SurfaceState::Unmeasured);
        assert!(m.surface().is_flat());

        let first = warped();
        let job = feed(&mut m, &first, 0.0, &mut rng).expect("enough views");
        assert_eq!(m.accept(job.run(&k())), Verdict::First);
        assert_eq!(m.state(), SurfaceState::Verified);
        assert!(m.surface().max_difference(&first, W, H) < 0.1);
        assert!(m.job(2.5).is_none(), "no fit before REFIT_S");

        // the same board, more views: confirmed
        let job = feed(&mut m, &first, 10.0, &mut rng).unwrap();
        assert!(matches!(m.accept(job.run(&k())), Verdict::Confirmed { .. }));

        // laid down anew, flat this time: suspect first, then replaced
        let flat = BoardSurface::flat();
        let job = feed(&mut m, &flat, 200.0, &mut rng).unwrap();
        assert!(matches!(m.accept(job.run(&k())), Verdict::Suspect { .. }));
        assert_eq!(m.state(), SurfaceState::Measuring);
        let job = feed(&mut m, &flat, 210.0, &mut rng).unwrap();
        let v = m.accept(job.run(&k()));
        assert!(matches!(v, Verdict::Replaced { .. }) && v.changed_shape(), "{v:?}");
        assert!(m.surface().peak_to_valley(W, H) < 0.1);
    }

    #[test]
    fn one_stray_fit_does_not_replace_the_shape() {
        let mut rng = Rng::new(5);
        let mut m = SurfaceMonitor::new(W, H, 3);
        m.restore(MeasuredSurface { surface: warped(), rms_px: 0.2, flat_rms_px: 0.4, height_sigma_mm: 0.03 });
        let job = feed(&mut m, &BoardSurface::flat(), 0.0, &mut rng).unwrap();
        assert!(matches!(m.accept(job.run(&k())), Verdict::Suspect { .. }));
        // the fresh views show the shape in use after all
        let job = feed(&mut m, &warped(), 10.0, &mut rng).unwrap();
        assert!(matches!(m.accept(job.run(&k())), Verdict::Confirmed { .. }));
        assert!(m.surface().max_difference(&warped(), W, H) < 1e-9);
    }

    #[test]
    fn a_poor_fit_changes_nothing() {
        let mut m = SurfaceMonitor::new(W, H, 3);
        assert_eq!(m.accept(None), Verdict::Undetermined);
        assert_eq!(m.state(), SurfaceState::Unmeasured);
    }
}
