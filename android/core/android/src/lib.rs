//! EasyScanner native core and its JNI bridge to
//! `com.rawmechatronics.easyscanner.NativeLib`.
//!
//! The app owns one engine handle and calls every function on the same
//! processing thread. Camera planes arrive as direct `ByteBuffer`s and are
//! read in place; the result is written into a caller-owned `float[]`.

mod board;
mod calib;
mod engine;
mod frame_out;

#[cfg(test)]
mod e2e_tests;

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::PathBuf;

use jni::JNIEnv;
use jni::objects::{JByteBuffer, JClass, JFloatArray, JLongArray, JString, ReleaseMode};
use jni::sys::{jint, jlong, jstring};
use scan_algo::imu::GyroSample;
use scan_algo::laser::{YuvFrame, YuvLayout};
use scan_algo::rolling::RowClock;

use engine::Engine;

/// Run `f`, turning an `Err` or a panic into a Java `RuntimeException`.
fn guard<'l, R>(env: &mut JNIEnv<'l>, fallback: R, f: impl FnOnce(&mut JNIEnv<'l>) -> Result<R, String>) -> R {
    let msg = match catch_unwind(AssertUnwindSafe(|| f(&mut *env))) {
        Ok(Ok(v)) => return v,
        Ok(Err(m)) => m,
        Err(p) => p
            .downcast_ref::<&str>()
            .map(|s| s.to_string())
            .or_else(|| p.downcast_ref::<String>().cloned())
            .unwrap_or_else(|| "native panic".into()),
    };
    log::error!("{msg}");
    let _ = env.throw_new("java/lang/RuntimeException", msg);
    fallback
}

/// # Safety
/// `handle` must come from `nativeCreate` and not yet be destroyed.
unsafe fn engine<'a>(handle: jlong) -> Result<&'a mut Engine, String> {
    unsafe { (handle as *mut Engine).as_mut() }.ok_or_else(|| "engine handle is null".to_string())
}

fn string_arg(env: &mut JNIEnv, s: &JString) -> Result<String, String> {
    if s.is_null() {
        return Ok(String::new());
    }
    env.get_string(s).map(Into::into).map_err(|e| e.to_string())
}

fn new_string(env: &mut JNIEnv, s: &str) -> Result<jstring, String> {
    env.new_string(s).map(|j| j.into_raw()).map_err(|e| e.to_string())
}

fn buffer<'b>(env: &mut JNIEnv, buf: &'b JByteBuffer) -> Result<&'b [u8], String> {
    let ptr = env.get_direct_buffer_address(buf).map_err(|e| e.to_string())?;
    let len = env.get_direct_buffer_capacity(buf).map_err(|e| e.to_string())?;
    // SAFETY: a direct buffer's memory stays valid while the Java caller holds
    // the Image it belongs to, i.e. for the duration of this call.
    Ok(unsafe { std::slice::from_raw_parts(ptr, len) })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_rawmechatronics_easyscanner_NativeLib_nativeCreate<'l>(
    mut env: JNIEnv<'l>,
    _cls: JClass<'l>,
    files_dir: JString<'l>,
) -> jlong {
    #[cfg(target_os = "android")]
    android_logger::init_once(android_logger::Config::default().with_max_level(log::LevelFilter::Info).with_tag("EasyScanner"));
    guard(&mut env, 0, |env| {
        let dir = PathBuf::from(string_arg(env, &files_dir)?);
        let e = Engine::new(dir).map_err(|e| e.to_string())?;
        Ok(Box::into_raw(Box::new(e)) as jlong)
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_rawmechatronics_easyscanner_NativeLib_nativeDestroy<'l>(_env: JNIEnv<'l>, _cls: JClass<'l>, handle: jlong) {
    if handle != 0 {
        // SAFETY: the handle came from Box::into_raw in nativeCreate.
        drop(unsafe { Box::from_raw(handle as *mut Engine) });
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_rawmechatronics_easyscanner_NativeLib_nativeConfigure<'l>(
    mut env: JNIEnv<'l>,
    _cls: JClass<'l>,
    handle: jlong,
    json: JString<'l>,
) {
    guard(&mut env, (), |env| {
        let json = string_arg(env, &json)?;
        unsafe { engine(handle) }?.configure(&json)
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_rawmechatronics_easyscanner_NativeLib_nativeCommand<'l>(
    mut env: JNIEnv<'l>,
    _cls: JClass<'l>,
    handle: jlong,
    code: jint,
    arg: JString<'l>,
) -> jstring {
    guard(&mut env, std::ptr::null_mut(), |env| {
        let arg = string_arg(env, &arg)?;
        let msg = unsafe { engine(handle) }?.command(code, &arg);
        new_string(env, &msg)
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_rawmechatronics_easyscanner_NativeLib_nativeMessages<'l>(
    mut env: JNIEnv<'l>,
    _cls: JClass<'l>,
    handle: jlong,
) -> jstring {
    guard(&mut env, std::ptr::null_mut(), |env| {
        let msg = unsafe { engine(handle) }?.messages();
        new_string(env, &msg)
    })
}

/// Process one YUV_420_888 frame (`timestamp_ns`: its sensor timestamp,
/// `readout_ns`: its rolling-shutter skew, `exposure_ns`: its exposure time;
/// 0 when unknown). Returns the number of floats written to `out`, or
/// `-needed` when `out` is too small (nothing written).
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub extern "system" fn Java_com_rawmechatronics_easyscanner_NativeLib_nativeProcess<'l>(
    mut env: JNIEnv<'l>,
    _cls: JClass<'l>,
    handle: jlong,
    y: JByteBuffer<'l>,
    y_stride: jint,
    u: JByteBuffer<'l>,
    v: JByteBuffer<'l>,
    uv_stride: jint,
    uv_pixel_stride: jint,
    width: jint,
    height: jint,
    timestamp_ns: jlong,
    readout_ns: jlong,
    exposure_ns: jlong,
    out: JFloatArray<'l>,
) -> jint {
    guard(&mut env, 0, |env| {
        let dim = |name: &str, v: jint| usize::try_from(v).map_err(|_| format!("negative frame {name} {v}"));
        let layout = YuvLayout {
            width: dim("width", width)?,
            height: dim("height", height)?,
            y_stride: dim("row stride", y_stride)?,
            uv_stride: dim("chroma row stride", uv_stride)?,
            uv_pixel_stride: dim("chroma pixel stride", uv_pixel_stride)?,
        };
        let frame = YuvFrame::new(layout, buffer(env, &y)?, buffer(env, &u)?, buffer(env, &v)?).map_err(|e| e.to_string())?;
        let clock = RowClock {
            start_ns: timestamp_ns,
            readout_ns: readout_ns.max(0),
            exposure_ns: exposure_ns.max(0),
            height: layout.height as f64,
        };
        let data = unsafe { engine(handle) }?.process(&frame, clock).map_err(|e| e.to_string())?;
        let cap = env.get_array_length(&out).map_err(|e| e.to_string())? as usize;
        if data.len() > cap {
            return Ok(-(data.len() as jint));
        }
        let n = data.len() as jint;
        env.set_float_array_region(&out, 0, data).map_err(|e| e.to_string())?;
        unsafe { engine(handle) }?.clear_events();
        Ok(n)
    })
}

/// Hand the engine `count` gyroscope samples: `timestamps_ns[i]` (the
/// `SensorEvent` clock) and the rate `rates[3i..3i+3]` (rad/s, device axes).
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_rawmechatronics_easyscanner_NativeLib_nativePushGyro<'l>(
    mut env: JNIEnv<'l>,
    _cls: JClass<'l>,
    handle: jlong,
    timestamps_ns: JLongArray<'l>,
    rates: JFloatArray<'l>,
    count: jint,
) {
    guard(&mut env, (), |env| {
        // SAFETY: the arrays are only read, and released (without copying
        // back) before this call returns; Kotlin does not touch them meanwhile
        let ts = unsafe { env.get_array_elements(&timestamps_ns, ReleaseMode::NoCopyBack) }.map_err(|e| e.to_string())?;
        let w = unsafe { env.get_array_elements(&rates, ReleaseMode::NoCopyBack) }.map_err(|e| e.to_string())?;
        let n = (count.max(0) as usize).min(ts.len()).min(w.len() / 3);
        let samples: Vec<GyroSample> =
            ts[..n].iter().zip(w.chunks_exact(3)).map(|(&t_ns, r)| GyroSample { t_ns, rate: [r[0], r[1], r[2]].map(f64::from) }).collect();
        unsafe { engine(handle) }?.push_gyro(&samples);
        Ok(())
    })
}
