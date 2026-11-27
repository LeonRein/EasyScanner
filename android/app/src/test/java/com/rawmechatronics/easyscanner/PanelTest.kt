package com.rawmechatronics.easyscanner

import com.rawmechatronics.easyscanner.FrameResult.H
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class PanelTest {

    @Test
    fun progressIsTheShareOfTheTarget() {
        assertEquals(0.35f, Progress.of(7, 20, "7 of 20").fraction!!, 1e-6f)
    }

    @Test
    fun progressPastTheTargetIsAFullBar() {
        assertEquals(1f, Progress.of(25, 20, "").fraction!!, 0f)
    }

    @Test
    fun progressWithoutATargetIsIndeterminate() {
        assertNull(Progress.of(3, 0, "").fraction)
    }

    private fun frame(vararg slots: Pair<Int, Float>) = FrameResult(H.LEN).apply {
        data.fill(Float.NaN)
        slots.forEach { (slot, v) -> data[slot] = v }
        length = H.LEN
    }

    private val context = DebugHud.Context(fps = 29.7, resolution = null, intrinsics = "saved", messages = listOf("calib: saved"))

    @Test
    fun debugHudShowsThePoseOnlyWithABoard() {
        val none = DebugHud.text(frame(H.DETECTED to 0f, H.N_CORNERS to 0f, H.N_MARKERS to 2f), context)
        assertTrue(none, none.contains("board: none (0 corners, 2 markers)"))
        assertFalse(none.contains("rpy"))

        val seen = DebugHud.text(frame(H.DETECTED to 1f, H.N_CORNERS to 24f, H.REPROJ_RMS to 0.4f), context)
        assertTrue(seen, seen.contains("board 24 pts  rms 0.40 px"))
        assertTrue(seen.contains("rpy"))
    }

    @Test
    fun debugHudLinesFitAPhoneWidePreview() {
        val text = DebugHud.text(
            frame(
                H.DETECTED to 1f, H.N_CORNERS to 24f, H.SCAN_ON to 1f, H.LASER_STATE to 2f,
                H.LASER_LENGTH to 123f, H.IN_VOLUME to 40f, H.PHOTO_STATE to FrameResult.Photo.HOLD_FEW_CORNERS.toFloat(),
            ),
            context,
        )
        text.lines().forEach { assertTrue("too long: '$it'", it.length <= MAX_HUD_LINE) }
        assertTrue(text.endsWith("calib: saved"))
    }

    private companion object {
        /** Characters of 11 sp monospace across a 360 dp preview, less the margins. */
        const val MAX_HUD_LINE = 48
    }
}
