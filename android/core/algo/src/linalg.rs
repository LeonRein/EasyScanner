//! Small dense linear algebra for the least-squares solvers.

/// Solve the dense `n×n` system `a·x = b` by Gaussian elimination with
/// partial pivoting. `None` when singular.
#[allow(clippy::needless_range_loop)]
pub fn solve_dense(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for c in 0..n {
        let p = (c..n).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-300 {
            return None;
        }
        a.swap(c, p);
        b.swap(c, p);
        for r in c + 1..n {
            let f = a[r][c] / a[c][c];
            if f != 0.0 {
                for k in c..n {
                    a[r][k] -= f * a[c][k];
                }
                b[r] -= f * b[c];
            }
        }
    }
    let mut x = vec![0.0; n];
    for r in (0..n).rev() {
        let s: f64 = (r + 1..n).map(|k| a[r][k] * x[k]).sum();
        x[r] = (b[r] - s) / a[r][r];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/// The inverse of the dense `n×n` matrix `a`, column by column. `None`
/// when singular.
pub fn invert(a: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
    let n = a.len();
    let mut inv = vec![vec![0.0; n]; n];
    for c in 0..n {
        let mut e = vec![0.0; n];
        e[c] = 1.0;
        let col = solve_dense(a.to_vec(), e)?;
        for r in 0..n {
            inv[r][c] = col[r];
        }
    }
    Some(inv)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dense_solver() {
        let x = solve_dense(vec![vec![2.0, 1.0], vec![1.0, 3.0]], vec![3.0, 5.0]).unwrap();
        assert!((x[0] - 0.8).abs() < 1e-12 && (x[1] - 1.4).abs() < 1e-12);
    }

    #[test]
    fn inverse() {
        let inv = invert(&[vec![2.0, 1.0], vec![1.0, 3.0]]).unwrap();
        assert!((inv[0][0] - 0.6).abs() < 1e-12 && (inv[0][1] + 0.2).abs() < 1e-12 && (inv[1][1] - 0.4).abs() < 1e-12);
        assert!(invert(&[vec![1.0, 2.0], vec![2.0, 4.0]]).is_none());
    }
}
