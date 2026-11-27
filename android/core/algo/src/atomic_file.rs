//! Crash-safe file writes: the content goes to a temporary file next to the
//! target, is synced to the device and then renamed over the target, so a
//! reader (or the next launch after a kill) finds either the old file or the
//! complete new one, never half of one.

use std::ffi::OsString;
use std::fs::{self, File};
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};

/// Write `path` through `write`, atomically. On any error the temporary file
/// is removed and the old content of `path` is left untouched.
pub fn write_atomic<T>(path: &Path, write: impl FnOnce(&mut BufWriter<File>) -> io::Result<T>) -> io::Result<T> {
    let tmp = tmp_path(path)?;
    let result = write_and_sync(&tmp, write).and_then(|value| fs::rename(&tmp, path).map(|()| value));
    if result.is_err() {
        // best effort: the original error is the one worth reporting
        let _ = fs::remove_file(&tmp);
    }
    result
}

/// [`write_atomic`] for content that is already in memory.
pub fn write_bytes_atomic(path: &Path, bytes: impl AsRef<[u8]>) -> io::Result<()> {
    write_atomic(path, |w| w.write_all(bytes.as_ref()))
}

fn write_and_sync<T>(tmp: &Path, write: impl FnOnce(&mut BufWriter<File>) -> io::Result<T>) -> io::Result<T> {
    let mut w = BufWriter::new(File::create(tmp)?);
    let value = write(&mut w)?;
    // the data must be on the device before the rename makes it visible
    w.into_inner().map_err(io::IntoInnerError::into_error)?.sync_all()?;
    Ok(value)
}

/// `<name>.tmp` beside `path` (same directory, so the rename stays on one
/// file system).
fn tmp_path(path: &Path) -> io::Result<PathBuf> {
    let name = path.file_name().ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, format!("{} names no file", path.display())))?;
    let mut tmp = OsString::from(name);
    tmp.push(".tmp");
    Ok(path.with_file_name(tmp))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("es-atomic-{name}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn writes_the_whole_file_and_leaves_no_temporary() {
        let dir = scratch("ok");
        let path = dir.join("a.json");
        write_bytes_atomic(&path, "old").unwrap();
        write_bytes_atomic(&path, "new").unwrap();
        assert_eq!(fs::read_to_string(&path).unwrap(), "new");
        assert!(!dir.join("a.json.tmp").exists());
    }

    #[test]
    fn a_failed_write_keeps_the_old_file() {
        let dir = scratch("fail");
        let path = dir.join("a.json");
        write_bytes_atomic(&path, "old").unwrap();
        let err = write_atomic(&path, |w| {
            w.write_all(b"half")?;
            Err::<(), _>(io::Error::other("killed"))
        });
        assert!(err.is_err());
        assert_eq!(fs::read_to_string(&path).unwrap(), "old");
        assert!(!dir.join("a.json.tmp").exists());
    }

    #[test]
    fn returns_the_writer_value() {
        let dir = scratch("value");
        let n = write_atomic(&dir.join("n.txt"), |w| w.write_all(b"12345").map(|()| 5)).unwrap();
        assert_eq!(n, 5);
    }
}
