//! The board's measured shape in the app: [`scan_algo::surface_monitor`]
//! with its fits run on their own thread, and the shape saved per board.

use std::path::Path;
use std::thread::JoinHandle;

use scan_algo::board_surface::{BoardSurface, SurfaceFit, SurfaceView};
use scan_algo::config::{self, BoardSpec, SurfaceCfg};
use scan_algo::geom::Intrinsics;
use scan_algo::surface_monitor::{MeasuredSurface, SurfaceMonitor, SurfaceState, Verdict};

/// The plane z = 0, for a disabled shape model.
static FLAT: BoardSurface = BoardSurface::flat();

pub struct SurfaceTracker {
    cfg: SurfaceCfg,
    board: BoardSpec,
    monitor: SurfaceMonitor,
    job: Option<JoinHandle<Option<SurfaceFit>>>,
}

impl SurfaceTracker {
    /// For `board`, starting from the shape saved for it in `dir`, if any.
    pub fn new(dir: &Path, board: BoardSpec, cfg: SurfaceCfg) -> Self {
        let mut monitor = SurfaceMonitor::new(board.width_mm(), board.height_mm(), cfg.degree);
        if let Some(saved) = config::load_surface(dir, &board) {
            monitor.restore(saved);
        }
        SurfaceTracker { cfg, board, monitor, job: None }
    }

    /// The settings changed: a new board or degree starts over (from the
    /// shape saved for that board).
    pub fn configure(&mut self, dir: &Path, board: BoardSpec, cfg: SurfaceCfg) {
        if board != self.board || cfg.degree != self.cfg.degree {
            *self = SurfaceTracker::new(dir, board, cfg);
        }
        self.cfg = cfg;
    }

    /// The shape the board's points are computed on.
    pub fn surface(&self) -> &BoardSurface {
        if self.cfg.enabled { self.monitor.surface() } else { &FLAT }
    }

    pub fn state(&self) -> SurfaceState {
        if self.cfg.enabled { self.monitor.state() } else { SurfaceState::Unmeasured }
    }

    /// The degree a shape fit should use; `None` when the model is off.
    pub fn degree(&self) -> Option<u8> {
        self.cfg.enabled.then_some(self.cfg.degree)
    }

    pub fn measured(&self) -> Option<&MeasuredSurface> {
        self.monitor.measured().filter(|_| self.cfg.enabled)
    }

    pub fn progress(&self) -> (usize, usize) {
        self.monitor.progress()
    }

    /// Offer a steady, well-posed frame's corners.
    pub fn offer(&mut self, t_s: f64, view: impl FnOnce() -> SurfaceView) {
        if self.cfg.enabled {
            self.monitor.offer(t_s, view());
        }
    }

    /// Drop the measured shape, saved one included, and measure anew.
    pub fn forget(&mut self, dir: &Path) -> std::io::Result<()> {
        self.job = None;
        self.monitor.reset();
        match std::fs::remove_file(dir.join(config::SURFACE_FILE)) {
            Err(e) if e.kind() != std::io::ErrorKind::NotFound => Err(e),
            _ => Ok(()),
        }
    }

    /// The views collected so far were taken through another camera model
    /// or at another resolution: drop them, and a fit in flight.
    pub fn restart_views(&mut self) {
        self.job = None;
        self.monitor.forget_views();
    }

    /// Weigh a shape measured elsewhere (by the camera calibration) like
    /// one of the tracker's own fits; a new shape is saved for the board in `dir`.
    pub fn propose(&mut self, dir: &Path, fit: SurfaceFit) -> Option<Settled> {
        self.cfg.enabled.then(|| self.settle(dir, Some(fit)))
    }

    /// Collect a finished fit and start the next one when due. `k` is the
    /// camera model of the views' frames. Returns what came of a finished
    /// fit; a new shape is saved for the board in `dir`.
    pub fn step(&mut self, dir: &Path, t_s: f64, k: &Intrinsics) -> Option<Settled> {
        if !self.cfg.enabled {
            return None;
        }
        let mut out = None;
        if self.job.as_ref().is_some_and(JoinHandle::is_finished) {
            let result = self.job.take().and_then(|j| j.join().ok()).flatten();
            out = Some(self.settle(dir, result));
        }
        if self.job.is_none()
            && let Some(job) = self.monitor.job(t_s)
        {
            let k = *k;
            self.job = Some(std::thread::spawn(move || job.run(&k)));
        }
        out
    }

    fn settle(&mut self, dir: &Path, result: Option<SurfaceFit>) -> Settled {
        let verdict = self.monitor.accept(result);
        let saved = match (verdict.changed_shape(), self.monitor.measured()) {
            (true, Some(m)) => config::save_surface(dir, &self.board, m),
            _ => Ok(()),
        };
        Settled { verdict, saved }
    }
}

/// What came of a fit: the verdict, and whether a new shape was saved.
pub struct Settled {
    pub verdict: Verdict,
    pub saved: std::io::Result<()>,
}
