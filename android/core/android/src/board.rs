//! ChArUco board detection and the camera-relative-to-board pose — the same
//! math as the desktop prototype's `detect.py` (CharucoDetector → matchImagePoints →
//! solvePnPGeneric/IPPE with temporal flip disambiguation), plus a tracking
//! ROI so a found board is searched for near where it was, and the
//! rolling-shutter correction of the corners (`scan_algo::rolling`).

use opencv::core::{self, Mat, Point2f, Point3f, Rect, Vector};
use opencv::prelude::*;
use opencv::{calib3d, objdetect};
use scan_algo::config::BoardSpec;
use scan_algo::geom::{Intrinsics, M3, Pose, V3, norm, rodrigues};
use scan_algo::ippe::{self, Branch};
use scan_algo::rolling::{Correction, solve_corrected};

pub fn make_board(spec: &BoardSpec) -> opencv::Result<objdetect::CharucoBoard> {
    let dict = objdetect::get_predefined_dictionary_i32(spec.dict)?;
    objdetect::CharucoBoard::new_def(core::Size::new(spec.squares_x, spec.squares_y), spec.square_mm, spec.marker_mm, &dict)
}

pub struct PoseResult {
    pub pose: Pose,
    pub reproj_rms: f64,
    /// Runner-up IPPE branch's error / the chosen one's: ≫1 unambiguous,
    /// ≈1 near-frontal and genuinely flippable.
    pub margin: Option<f64>,
}

pub struct BoardTracker {
    pub spec: BoardSpec,
    board: objdetect::CharucoBoard,
    detector: objdetect::CharucoDetector,
    roi_tracking: bool,
    prev_r: Option<M3>,
    /// Next search window and the frame size it was computed for: a window
    /// from another resolution (the governor switched) is meaningless.
    search: Option<(Rect, (i32, i32))>,
    frames_since_full: u32,
    // detections of the current frame, full-frame pixel coordinates
    pub corners: Vector<Point2f>,
    pub ids: Vector<i32>,
    marker_corners: Vector<Vector<Point2f>>,
    marker_ids: Vector<i32>,
    obj: Vector<Point3f>,
    img: Vector<Point2f>,
}

impl BoardTracker {
    pub fn new(spec: BoardSpec, aruco3: bool, roi_tracking: bool) -> opencv::Result<Self> {
        let board = make_board(&spec)?;
        let mut params = objdetect::DetectorParameters::default()?;
        // sub-pixel marker corners feed the ChArUco interpolation and the pose
        params.set_corner_refinement_method(objdetect::CornerRefineMethod::CORNER_REFINE_SUBPIX as i32);
        params.set_use_aruco3_detection(aruco3);
        let detector = objdetect::CharucoDetector::new(
            &board,
            &objdetect::CharucoParameters::default()?,
            &params,
            objdetect::RefineParameters::new_def()?,
        )?;
        Ok(BoardTracker {
            spec,
            board,
            detector,
            roi_tracking,
            prev_r: None,
            search: None,
            frames_since_full: 0,
            corners: Vector::new(),
            ids: Vector::new(),
            marker_corners: Vector::new(),
            marker_ids: Vector::new(),
            obj: Vector::new(),
            img: Vector::new(),
        })
    }

    /// Detect the board's ChArUco corners in a luma image. Tries the tracking
    /// window first and falls back to the full frame on a miss (and every
    /// 15th frame, to pick up parts of the board that just came into view).
    pub fn detect(&mut self, gray: &Mat) -> opencv::Result<usize> {
        let (w, h) = (gray.cols(), gray.rows());
        let windowed = self.roi_tracking && self.frames_since_full < 15;
        let window = self.search.take().filter(|&(_, size)| size == (w, h)).map(|(r, _)| r);
        if let (true, Some(r)) = (windowed, window) {
            let sub = Mat::roi(gray, r)?;
            self.detect_in(&sub)?;
            if self.corners.len() >= 4 {
                for i in 0..self.corners.len() {
                    let mut p = self.corners.get(i)?;
                    p.x += r.x as f32;
                    p.y += r.y as f32;
                    self.corners.set(i, p)?;
                }
                self.frames_since_full += 1;
                self.update_search(w, h)?;
                return Ok(self.corners.len());
            }
        }
        self.detect_in(gray)?;
        self.frames_since_full = 0;
        self.update_search(w, h)?;
        Ok(self.corners.len())
    }

    fn detect_in(&mut self, img: &impl core::ToInputArray) -> opencv::Result<()> {
        self.corners.clear();
        self.ids.clear();
        self.marker_corners.clear();
        self.marker_ids.clear();
        self.detector.detect_board(img, &mut self.corners, &mut self.ids, &mut self.marker_corners, &mut self.marker_ids)
    }

    /// Next frame's search window: the corners' bounding box grown by a
    /// border-square's worth (markers extend past the outer corners) plus
    /// room for motion.
    fn update_search(&mut self, w: i32, h: i32) -> opencv::Result<()> {
        if self.corners.len() < 4 {
            self.search = None;
            return Ok(());
        }
        let (mut x0, mut y0, mut x1, mut y1) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
        for p in self.corners.iter() {
            x0 = x0.min(p.x);
            y0 = y0.min(p.y);
            x1 = x1.max(p.x);
            y1 = y1.max(p.y);
        }
        let (bw, bh) = (x1 - x0, y1 - y0);
        let inner = ((self.spec.squares_x - 2).max(1)).min((self.spec.squares_y - 2).max(1)) as f32;
        let square = bw.max(bh) / inner;
        let pad = 1.5 * square + 0.25 * bw.max(bh) + 16.0;
        let rx0 = ((x0 - pad).floor() as i32).clamp(0, w - 1);
        let ry0 = ((y0 - pad).floor() as i32).clamp(0, h - 1);
        let rx1 = ((x1 + pad).ceil() as i32).clamp(rx0 + 1, w);
        let ry1 = ((y1 + pad).ceil() as i32).clamp(ry0 + 1, h);
        // a window covering most of the frame buys nothing
        self.search = ((rx1 - rx0) * (ry1 - ry0) < w * h * 3 / 4).then(|| (Rect::new(rx0, ry0, rx1 - rx0, ry1 - ry0), (w, h)));
        Ok(())
    }

    /// ArUco markers of the current frame: many markers but no corners means
    /// the board settings do not match the printed board.
    pub fn n_markers(&self) -> usize {
        self.marker_ids.len()
    }

    /// Board→camera pose from the current detection.
    ///
    /// `expected` is the rotation the camera should have now (the last pose
    /// turned on by the gyro); without it the last pose this tracker solved
    /// decides IPPE's near-frontal ambiguity. `corr` undoes the rolling
    /// shutter: the pose is then the one at the frame's middle row.
    pub fn solve_pose(&mut self, k: &Intrinsics, expected: Option<&M3>, corr: Option<&Correction>) -> opencv::Result<Option<PoseResult>> {
        if self.corners.len() < 4 {
            return Ok(None);
        }
        self.obj.clear();
        self.img.clear();
        self.board.match_image_points(&self.corners, &self.ids, &mut self.obj, &mut self.img)?;
        if self.obj.len() < 4 {
            return Ok(None);
        }
        let obj: Vec<V3> = self.obj.iter().map(|p| [p.x as f64, p.y as f64, p.z as f64]).collect();
        let img: Vec<[f64; 2]> = self.img.iter().map(|p| [p.x as f64, p.y as f64]).collect();
        let expected = expected.copied().or(self.prev_r);
        let obj_cv = &self.obj;
        let res = solve_corrected(corr, k, &obj, &img, |px| solve_planar(obj_cv, k, &obj, px, expected.as_ref()), |r| r.pose)?;
        if let Some(r) = &res {
            self.prev_r = Some(r.pose.r);
        }
        Ok(res)
    }
}

/// IPPE on the board points and their (possibly corrected) pixels, with the
/// branch chosen by [`ippe::choose`]; plain iterative PnP (no margin) when
/// IPPE refuses the view.
fn solve_planar(
    obj_cv: &Vector<Point3f>,
    k: &Intrinsics,
    obj: &[V3],
    px: &[[f64; 2]],
    expected: Option<&M3>,
) -> opencv::Result<Option<PoseResult>> {
    let k_mat = Mat::from_slice_2d(&[[k.fx, 0.0, k.cx], [0.0, k.fy, k.cy], [0.0, 0.0, 1.0]])?;
    let d_mat = Mat::from_slice(&k.dist)?;
    let img: Vector<Point2f> = px.iter().map(|p| Point2f::new(p[0] as f32, p[1] as f32)).collect();

    let mut rvecs: Vector<Mat> = Vector::new();
    let mut tvecs: Vector<Mat> = Vector::new();
    let solutions = calib3d::solve_pnp_generic(
        obj_cv,
        &img,
        &k_mat,
        &d_mat,
        &mut rvecs,
        &mut tvecs,
        false,
        calib3d::SolvePnPMethod::SOLVEPNP_IPPE,
        &core::no_array(),
        &core::no_array(),
        &mut core::no_array(),
    )
    .unwrap_or(0);

    let n = (solutions.max(0) as usize).min(rvecs.len());
    let poses = (0..n).map(|i| pose_from(&rvecs.get(i)?, &tvecs.get(i)?)).collect::<opencv::Result<Vec<Pose>>>()?;
    let branches: Vec<Branch> = poses.iter().map(|p| Branch { rotation: p.r, rms: reproj_rms(k, p, obj, px) }).collect();
    if let Some(choice) = ippe::choose(&branches, expected) {
        let i = choice.index;
        return Ok(Some(PoseResult { pose: poses[i], reproj_rms: branches[i].rms, margin: Some(choice.margin) }));
    }
    // IPPE refused the view — plain iterative PnP, no margin
    let (mut rvec, mut tvec) = (Mat::default(), Mat::default());
    if !calib3d::solve_pnp(obj_cv, &img, &k_mat, &d_mat, &mut rvec, &mut tvec, false, calib3d::SOLVEPNP_ITERATIVE)? {
        return Ok(None);
    }
    let pose = pose_from(&rvec, &tvec)?;
    Ok(Some(PoseResult { pose, reproj_rms: reproj_rms(k, &pose, obj, px), margin: None }))
}

fn pose_from(rvec: &Mat, tvec: &Mat) -> opencv::Result<Pose> {
    let r = [*rvec.at::<f64>(0)?, *rvec.at::<f64>(1)?, *rvec.at::<f64>(2)?];
    let t = [*tvec.at::<f64>(0)?, *tvec.at::<f64>(1)?, *tvec.at::<f64>(2)?];
    Ok(Pose { r: rodrigues(r), t })
}

/// Per-point reprojection RMS (px).
fn reproj_rms(k: &Intrinsics, pose: &Pose, obj: &[V3], img: &[[f64; 2]]) -> f64 {
    let mut ss = 0.0;
    let mut n = 0usize;
    for (o, i) in obj.iter().zip(img) {
        if let Some(p) = k.project(pose.to_cam(*o)) {
            ss += (p[0] - i[0]).powi(2) + (p[1] - i[1]).powi(2);
            n += 1;
        }
    }
    if n == 0 { f64::INFINITY } else { (ss / n as f64).sqrt() }
}

/// Bounding box of the detected corners `(x0, y0, x1, y1)` in pixels.
pub fn corner_bbox(corners: &Vector<Point2f>) -> Option<(usize, usize, usize, usize)> {
    if corners.is_empty() {
        return None;
    }
    let (mut x0, mut y0, mut x1, mut y1) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
    for p in corners.iter() {
        x0 = x0.min(p.x);
        y0 = y0.min(p.y);
        x1 = x1.max(p.x);
        y1 = y1.max(p.y);
    }
    Some((x0.max(0.0) as usize, y0.max(0.0) as usize, x1.ceil().max(0.0) as usize, y1.ceil().max(0.0) as usize))
}

/// Distance from the camera to the board origin (mm).
pub fn distance(pose: &Pose) -> f64 {
    norm(pose.t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use scan_algo::geom::{mat_mul, rotation_distance, scale};
    use scan_algo::ippe::AMBIGUOUS_MARGIN;

    /// A small patch of board corners seen nearly head-on from afar, with
    /// pixel noise: the textbook case of IPPE's two-fold ambiguity.
    #[test]
    fn expected_rotation_steers_an_ambiguous_ippe_view() {
        let k = Intrinsics { fx: 920.0, fy: 920.0, cx: 639.5, cy: 479.5, dist: [0.0; 5], width: 1280.0, height: 960.0 };
        let obj: Vec<V3> = (0..5).flat_map(|i| (0..5).map(move |j| [15.0 * i as f64, 15.0 * j as f64, 0.0])).collect();
        let obj_cv: Vector<Point3f> = obj.iter().map(|p| Point3f::new(p[0] as f32, p[1] as f32, p[2] as f32)).collect();
        // camera 700 mm above the patch, tilted 6° off the normal
        let r = mat_mul(&rodrigues([0.1, 0.0, 0.0]), &rodrigues([std::f64::consts::PI, 0.0, 0.0]));
        let truth = Pose { r, t: [-30.0, 30.0, 700.0] };
        // deterministic pixel noise (σ ≈ 0.3 px)
        let mut state = 0x2545_f491_u64;
        let mut noise = || {
            state = state.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            ((state >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * 1.04
        };
        let px: Vec<[f64; 2]> = obj
            .iter()
            .map(|&o| {
                let [u, v] = k.project(truth.to_cam(o)).unwrap();
                [u + noise(), v + noise()]
            })
            .collect();

        let unguided = solve_planar(&obj_cv, &k, &obj, &px, None).unwrap().unwrap();
        let margin = unguided.margin.unwrap();
        assert!(margin < AMBIGUOUS_MARGIN, "the view must be ambiguous: margin {margin:.2}");
        // the mirrored branch: tilted the other way
        let mirrored = mat_mul(&rodrigues(scale([1.0, 0.0, 0.0], -0.1)), &rodrigues([std::f64::consts::PI, 0.0, 0.0]));
        for (expected, name) in [(truth.r, "truth"), (mirrored, "mirror")] {
            let got = solve_planar(&obj_cv, &k, &obj, &px, Some(&expected)).unwrap().unwrap();
            let off = rotation_distance(&got.pose.r, &expected).to_degrees();
            assert!(off < 2.0, "expected the {name} branch, got one {off:.1}° away from it");
        }
    }
}
