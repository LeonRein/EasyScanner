package com.rawmechatronics.easyscanner

import com.rawmechatronics.easyscanner.FrameResult.H
import com.rawmechatronics.easyscanner.FrameResult.Photo

/**
 * The debug overlay's text: every number the core reports for a frame, in
 * short lines that fit over a phone-wide preview. Shown only with
 * Settings › Debug overlay; the guided panel never needs these numbers.
 * Pure formatting, no Android calls.
 */
object DebugHud {

    /** What the overlay shows besides the frame itself. */
    data class Context(
        val fps: Double,
        /** The resolution governor's state, or null at a fixed resolution. */
        val resolution: String?,
        /** Where the intrinsics came from (the core's "intrinsics" message). */
        val intrinsics: String,
        /** The core's latest messages, one per line. */
        val messages: List<String>,
    )

    fun text(r: FrameResult, c: Context): String {
        fun f(slot: Int) = r[slot]
        fun has(slot: Int) = !f(slot).isNaN()
        return buildString {
            appendLine("%.0fx%.0f  %.1f fps  %.1f ms".format(f(H.FRAME_W), f(H.FRAME_H), c.fps, r.procMs))
            appendLine("  board %.1f ms  laser %.1f ms".format(f(H.T_BOARD_MS), f(H.T_LASER_MS)))
            appendLine(c.resolution?.let { "auto: $it" } ?: "fixed resolution")
            if (r.flag(H.DETECTED)) {
                appendLine("board %d pts  rms %.2f px".format(f(H.N_CORNERS).toInt(), f(H.REPROJ_RMS)))
                appendLine("  margin %s  focus %s".format(
                    marginLabel(f(H.POSE_MARGIN)), if (has(H.FOCUS_PCT)) "%.0f%%".format(f(H.FOCUS_PCT)) else "-"))
                appendLine("cam %.1f %.1f %.1f mm  d %.0f".format(f(H.CAM_X), f(H.CAM_Y), f(H.CAM_Z), f(H.DISTANCE)))
                appendLine("  rpy %.1f %.1f %.1f deg".format(f(H.ROLL), f(H.PITCH), f(H.YAW)))
            } else {
                appendLine("board: none (%d corners, %d markers)".format(f(H.N_CORNERS).toInt(), f(H.N_MARKERS).toInt()))
            }
            appendLine("K %s  fx %.0f c %.0f,%.0f".format(c.intrinsics, f(H.FX), f(H.CX), f(H.CY)))
            when (f(H.LASER_STATE).toInt()) {
                FrameResult.Laser.OFF -> appendLine("laser: off")
                FrameResult.Laser.NONE -> appendLine("laser: none (peak %.0f)".format(f(H.LASER_PEAK)))
                else -> {
                    appendLine("laser %d pts  peak %.0f".format(f(H.LASER_POINTS).toInt(), f(H.LASER_PEAK)))
                    if (has(H.LASER_LENGTH)) {
                        appendLine("  on board %.0f mm @ %.0f deg".format(f(H.LASER_LENGTH), f(H.LASER_ANGLE)))
                    }
                    if (f(H.IN_VOLUME) > 0f) appendLine("  in volume %d".format(f(H.IN_VOLUME).toInt()))
                }
            }
            val plane = when {
                r.flag(H.PLANE_OK) -> "ok"
                r.flag(H.PLANE_STALE) -> "STALE"
                else -> "none"
            }
            appendLine("plane %s  cloud %d pts %.1f MB".format(plane, f(H.CLOUD_POINTS).toInt(), f(H.CLOUD_MB)))
            if (r.flag(H.CALIB_ACTIVE)) {
                appendLine("calib %d/%d  rms %s".format(
                    f(H.CALIB_COUNT).toInt(), f(H.CALIB_TARGET).toInt(),
                    if (has(H.CALIB_RMS)) "%.2f px".format(f(H.CALIB_RMS)) else "-"))
            }
            if (r.flag(H.PLANE_ACTIVE)) {
                appendLine("lines %d/%d  spread %.2f  rms %s".format(
                    f(H.PLANE_LINES).toInt(), f(H.PLANE_TARGET).toInt(), f(H.PLANE_SPREAD),
                    if (has(H.PLANE_RMS)) "%.2f mm".format(f(H.PLANE_RMS)) else "-"))
            }
            if (r.flag(H.SCAN_ON)) {
                append("kept %d  rejected %d".format(f(H.GATE_KEPT).toInt(), f(H.GATE_REJECTED).toInt()))
                if (has(H.SCAN_GATE)) append("  now %s".format(FrameResult.Gate.label(f(H.SCAN_GATE).toInt())))
                appendLine()
                append("gyro %s".format(FrameResult.Gyro.label(f(H.GYRO_STATE).toInt())))
                if (r.flag(H.RS_CORRECTED)) append("  RS corrected")
                appendLine()
            }
            val photos = f(H.PHOTO_STATE).toInt()
            if (photos != Photo.OFF) {
                appendLine("photos %d  %s".format(f(H.PHOTO_COUNT).toInt(), photoStateLabel(photos)))
                if (has(H.CAM_SPEED)) appendLine("  moving %.0f mm/s %.1f deg/s".format(f(H.CAM_SPEED), f(H.CAM_TURN)))
            }
            c.messages.forEach { appendLine(it) }
        }.trimEnd()
    }

    /** IPPE's branch margin: "-" without one, "∞" for a single branch. */
    fun marginLabel(m: Float) = when {
        m.isNaN() -> "-"
        m.isInfinite() -> "∞"
        else -> "%.1fx".format(m)
    }

    /** The photo keyframe state in the core's terms. */
    fun photoStateLabel(state: Int) = when (state) {
        Photo.WAITING -> "waiting"
        Photo.IN_FLIGHT -> "taking..."
        Photo.HOLD_NO_BOARD -> "due: no board"
        Photo.HOLD_POSE_RMS -> "due: pose rms high"
        Photo.HOLD_FEW_CORNERS -> "due: too few board corners"
        Photo.HOLD_SETTLING -> "due: measuring motion"
        Photo.HOLD_MOVING -> "due: hold steady"
        Photo.NO_PHOTOS -> "scan only"
        else -> "?"
    }
}
