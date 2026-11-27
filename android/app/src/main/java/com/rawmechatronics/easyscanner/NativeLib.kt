package com.rawmechatronics.easyscanner

import java.nio.ByteBuffer

/**
 * JNI surface of the Rust core (`core/android/src/lib.rs`). Every call for one
 * handle must come from the same thread — [FrameProcessor]'s processing thread.
 */
internal object NativeLib {
    init {
        System.loadLibrary("easyscanner_core")
    }

    @JvmStatic external fun nativeCreate(filesDir: String): Long
    @JvmStatic external fun nativeDestroy(handle: Long)
    @JvmStatic external fun nativeConfigure(handle: Long, json: String)
    @JvmStatic external fun nativeCommand(handle: Long, code: Int, arg: String?): String
    @JvmStatic external fun nativeMessages(handle: Long): String

    /**
     * Process one YUV_420_888 frame in place ([timestampNs]: its sensor
     * timestamp, [readoutNs]: its rolling-shutter skew, [exposureNs]: its
     * exposure time; 0 when unknown). Returns the number of floats written
     * to [out], or `-needed` when [out] is too small.
     */
    @JvmStatic external fun nativeProcess(
        handle: Long,
        y: ByteBuffer, yStride: Int,
        u: ByteBuffer, v: ByteBuffer, uvStride: Int, uvPixelStride: Int,
        width: Int, height: Int, timestampNs: Long, readoutNs: Long, exposureNs: Long,
        out: FloatArray,
    ): Int

    /**
     * Hand the core [count] gyroscope samples: [timestampsNs] on the
     * `SensorEvent` clock, [rates] as x, y, z per sample (rad/s, device axes).
     */
    @JvmStatic external fun nativePushGyro(handle: Long, timestampsNs: LongArray, rates: FloatArray, count: Int)
}

/** Engine commands; codes mirror `engine::cmd`. */
enum class Cmd(val code: Int) {
    CALIB_START(1), CALIB_STOP(2), CALIB_CAPTURE(3), CALIB_RESET(4),
    PLANE_START(5), PLANE_STOP(6), PLANE_FORCE(7),
    SCAN_START(8), SCAN_STOP(9), CLOUD_CLEAR(10), EXPORT_PLY(11),
    FORGET_INTRINSICS(12), FORGET_PLANE(13),
    PHOTOS_START(14), PHOTOS_STOP(15), PHOTO_SNAP(16), PHOTO_ADD(17), PHOTO_FAILED(18),
    CHECKPOINT(19), PHOTOS_OFF(20),
}
