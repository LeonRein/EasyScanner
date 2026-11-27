//! IPPE's two-fold ambiguity: a planar target seen nearly head-on fits two
//! mirror-image tilts almost equally well, and the better-fitting one can
//! flip from frame to frame. Which branch to trust is decided here, from the
//! fit and from where the camera is expected to be turned.

use crate::geom::{M3, rotation_distance};

/// Runner-up error / best error below this: the fit alone cannot tell the
/// branches apart.
pub const AMBIGUOUS_MARGIN: f64 = 3.0;
/// … and the expected branch is taken only while it fits at most this much
/// worse than the best one.
pub const MAX_FIT_RATIO: f64 = 2.0;

/// One of IPPE's solutions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Branch {
    /// Board→camera rotation.
    pub rotation: M3,
    /// Reprojection RMS (px).
    pub rms: f64,
}

/// The chosen branch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Choice {
    /// Index into the branches.
    pub index: usize,
    /// Runner-up reprojection error over the best one: ≫ 1 is unambiguous,
    /// infinite with a single branch.
    pub margin: f64,
}

/// Pick one of IPPE's branches. `expected` is the rotation the camera
/// should have now: the previous pose, or better, the previous pose turned
/// on by the gyro. `None` without branches.
pub fn choose(branches: &[Branch], expected: Option<&M3>) -> Option<Choice> {
    let by_fit = |a: &usize, b: &usize| branches[*a].rms.total_cmp(&branches[*b].rms);
    let best = (0..branches.len()).min_by(by_fit)?;
    let runner_up = (0..branches.len()).filter(|&i| i != best).min_by(by_fit);
    let margin = runner_up.map_or(f64::INFINITY, |i| branches[i].rms / branches[best].rms.max(1e-6));
    let index = match expected {
        Some(r) if margin < AMBIGUOUS_MARGIN => {
            let distance = |i: usize| rotation_distance(r, &branches[i].rotation);
            let nearest = (0..branches.len()).min_by(|&a, &b| distance(a).total_cmp(&distance(b))).unwrap_or(best);
            if branches[nearest].rms <= branches[best].rms * MAX_FIT_RATIO { nearest } else { best }
        }
        _ => best,
    };
    Some(Choice { index, margin })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::rodrigues;

    fn branch(r: [f64; 3], rms: f64) -> Branch {
        Branch { rotation: rodrigues(r), rms }
    }

    #[test]
    fn clear_fit_wins_regardless_of_expectation() {
        let (a, b) = (branch([3.0, 0.2, 0.0], 0.3), branch([3.0, -0.2, 0.0], 1.5));
        let c = choose(&[a, b], Some(&b.rotation)).unwrap();
        assert_eq!(c, Choice { index: 0, margin: 5.0 });
    }

    #[test]
    fn ambiguous_fit_follows_the_expected_rotation() {
        let (a, b) = (branch([3.0, 0.2, 0.0], 0.30), branch([3.0, -0.2, 0.0], 0.33));
        let near_b = rodrigues([3.0, -0.18, 0.01]);
        assert_eq!(choose(&[a, b], Some(&near_b)).unwrap().index, 1);
        assert_eq!(choose(&[a, b], None).unwrap().index, 0);
    }

    #[test]
    fn single_or_no_branch() {
        assert_eq!(choose(&[branch([0.1, 0.0, 0.0], 0.4)], None).unwrap().margin, f64::INFINITY);
        assert!(choose(&[], None).is_none());
    }
}
