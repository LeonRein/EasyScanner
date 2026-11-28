package com.rawmechatronics.easyscanner

import android.content.Context
import com.rawmechatronics.easyscanner.FrameResult.Calib

/**
 * The calibration's part of the bottom panel, the same in the guided step
 * and in expert mode: its progress, what needs the user's attention, and
 * the main button.
 */
class CameraCalibrationPanel(private val ctx: Context) {

    fun progress(s: CalibrationStatus): Progress? = when (s.state) {
        Calib.SOLVING -> Progress(null, ctx.getString(R.string.camera_solving))
        Calib.SAVED -> Progress(1f, ctx.getString(R.string.camera_saved))
        Calib.IDLE -> null
        else -> if (s.target > 0) {
            Progress.of(s.count, s.target, ctx.getString(R.string.camera_progress, s.count, s.target))
        } else null
    }

    fun notice(s: CalibrationStatus): Notice? = when {
        s.state == Calib.REJECTED -> Notice(rejectionText(s), Notice.Level.WARNING)
        s.needsTilt -> Notice(ctx.getString(R.string.camera_needs_tilt, sidesText(s.missingSides)), Notice.Level.INFO)
        else -> null
    }

    fun primary(s: CalibrationStatus, send: (Cmd) -> Unit): PanelAction {
        val (label, icon) = when (s.action) {
            CalibrationStatus.Action.START -> R.string.act_start to R.drawable.ic_play_arrow
            CalibrationStatus.Action.PAUSE -> R.string.act_pause to R.drawable.ic_pause
            CalibrationStatus.Action.WAIT -> R.string.act_pause to R.drawable.ic_pause
            CalibrationStatus.Action.RESUME -> R.string.act_resume to R.drawable.ic_play_arrow
            CalibrationStatus.Action.ADD_VIEWS -> R.string.act_add_views to R.drawable.ic_play_arrow
            CalibrationStatus.Action.START_OVER -> R.string.act_start_over to R.drawable.ic_play_arrow
            CalibrationStatus.Action.START_AGAIN -> R.string.act_calibrate_again to R.drawable.ic_play_arrow
        }
        return PanelAction(ctx.getString(label), enabled = s.action != CalibrationStatus.Action.WAIT, icon = icon) {
            send(if (s.action == CalibrationStatus.Action.PAUSE) Cmd.CALIB_STOP else Cmd.CALIB_START)
        }
    }

    /** Starting over by hand, offered while there are views to drop. */
    fun reset(s: CalibrationStatus, send: (Cmd) -> Unit): PanelAction? =
        if (s.count > 0 && s.state != Calib.SOLVING) {
            PanelAction(ctx.getString(R.string.act_start_over)) { send(Cmd.CALIB_RESET) }
        } else null

    private fun rejectionText(s: CalibrationStatus): String {
        val why = ctx.getString(
            when (s.rejection) {
                Calib.REJECT_TOO_FEW_VIEWS -> R.string.camera_rejected_few_views
                Calib.REJECT_HIGH_RMS -> R.string.camera_rejected_rms
                Calib.REJECT_IMPLAUSIBLE -> R.string.camera_rejected_implausible
                Calib.REJECT_FOCAL_UNCERTAIN -> R.string.camera_rejected_focal
                else -> R.string.camera_rejected_failed
            }
        )
        val next = ctx.getString(if (s.exhausted) R.string.camera_rejected_start_over else R.string.camera_rejected_add_views)
        return "$why $next"
    }

    /** "the left and from above" for the missing side bits. */
    private fun sidesText(bits: Int): String {
        val names = listOf(
            Calib.SIDE_LEFT to R.string.side_left,
            Calib.SIDE_RIGHT to R.string.side_right,
            Calib.SIDE_ABOVE to R.string.side_above,
            Calib.SIDE_BELOW to R.string.side_below,
        ).filter { (bit, _) -> bits and bit != 0 }.map { (_, name) -> ctx.getString(name) }
        return when (names.size) {
            0 -> ""
            1 -> names[0]
            else -> ctx.getString(R.string.list_and, names.dropLast(1).joinToString(", "), names.last())
        }
    }
}
