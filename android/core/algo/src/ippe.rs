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
/// Margin reported when IPPE returned a single solution.
pub const SINGLE_SOLUTION_MARGIN: f64 = 99.0;

/// The chosen branch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Choice {
    pub index: usize,
    /// Runner-up reprojection error over the best one (≫ 1 unambiguous).
    pub margin: f64,
}

/// Pick one of IPPE's solutions, given as (board→camera rotation,
/// reprojection RMS). `expected` is the rotation the camera should have now:
/// the previous pose, or better, the previous pose turned on by the gyro.
pub fn choose(cands: &[(M3, f64)], expected: Option<&M3>) -> Option<Choice> {
    let mut order: Vec<usize> = (0..cands.len()).collect();
    order.sort_by(|&a, &b| cands[a].1.total_cmp(&cands[b].1));
    let &best = order.first()?;
    let margin = order.get(1).map_or(SINGLE_SOLUTION_MARGIN, |&second| cands[second].1 / cands[best].1.max(1e-6));
    let index = match expected {
        Some(r) if cands.len() > 1 && margin < AMBIGUOUS_MARGIN => {
            let nearest = (0..cands.len())
                .min_by(|&a, &b| rotation_distance(r, &cands[a].0).total_cmp(&rotation_distance(r, &cands[b].0)))
                .unwrap_or(best);
            if cands[nearest].1 <= cands[best].1 * MAX_FIT_RATIO { nearest } else { best }
        }
        _ => best,
    };
    Some(Choice { index, margin })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::rodrigues;

    #[test]
    fn clear_fit_wins_regardless_of_expectation() {
        let (a, b) = (rodrigues([3.0, 0.2, 0.0]), rodrigues([3.0, -0.2, 0.0]));
        let c = choose(&[(a, 0.3), (b, 1.5)], Some(&b)).unwrap();
        assert_eq!(c, Choice { index: 0, margin: 5.0 });
    }

    #[test]
    fn ambiguous_fit_follows_the_expected_rotation() {
        let (a, b) = (rodrigues([3.0, 0.2, 0.0]), rodrigues([3.0, -0.2, 0.0]));
        let near_b = rodrigues([3.0, -0.18, 0.01]);
        assert_eq!(choose(&[(a, 0.30), (b, 0.33)], Some(&near_b)).unwrap().index, 1);
        assert_eq!(choose(&[(a, 0.30), (b, 0.33)], None).unwrap().index, 0);
    }

    #[test]
    fn single_or_no_solution() {
        assert_eq!(choose(&[(rodrigues([0.1, 0.0, 0.0]), 0.4)], None).unwrap().margin, SINGLE_SOLUTION_MARGIN);
        assert!(choose(&[], None).is_none());
    }
}
