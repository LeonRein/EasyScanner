package com.rawmechatronics.easyscanner

import com.rawmechatronics.easyscanner.FrameResult.Calib
import com.rawmechatronics.easyscanner.FrameResult.H

/**
 * The camera calibration as the core reports it (`calib::LiveCalibrator`),
 * read from one frame's header.
 */
data class CalibrationStatus(
    /** A [Calib] state. */
    val state: Int,
    val count: Int,
    val target: Int,
    /** A [Calib] `REJECT_*` reason, 0 unless [state] is [Calib.REJECTED]. */
    val rejection: Int,
    /** [Calib] `SIDE_*` bits: the sides still short of tilted views. */
    val missingSides: Int,
    /** The set is full: a rejection can only start over. */
    val exhausted: Boolean,
) {
    val collecting get() = state == Calib.COLLECTING

    /** Views are held for a solve to come: the frame size must not change. */
    val holdsViews get() = count > 0 && state in listOf(Calib.COLLECTING, Calib.SOLVING, Calib.PAUSED, Calib.REJECTED)

    /** Collecting, but only tilted views are still missing. */
    val needsTilt get() = collecting && count >= target && missingSides != 0 && !exhausted

    /** What the main button does now. */
    val action: Action
        get() = when (state) {
            Calib.COLLECTING -> Action.PAUSE
            Calib.SOLVING -> Action.WAIT
            Calib.PAUSED -> Action.RESUME
            Calib.REJECTED -> if (exhausted) Action.START_OVER else Action.ADD_VIEWS
            Calib.SAVED -> Action.START_AGAIN
            else -> Action.START
        }

    /** The main button's choices; every one but [WAIT] sends a command. */
    enum class Action { START, PAUSE, WAIT, RESUME, ADD_VIEWS, START_OVER, START_AGAIN }

    companion object {
        fun of(r: FrameResult) = CalibrationStatus(
            state = r[H.CALIB_STATE].toInt(),
            count = r[H.CALIB_COUNT].toInt(),
            target = r[H.CALIB_TARGET].toInt(),
            rejection = r[H.CALIB_REJECTION].toInt(),
            missingSides = r[H.CALIB_MISSING_SIDES].toInt(),
            exhausted = r.flag(H.CALIB_EXHAUSTED),
        )
    }
}
