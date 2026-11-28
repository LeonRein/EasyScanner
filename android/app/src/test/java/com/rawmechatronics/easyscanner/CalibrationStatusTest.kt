package com.rawmechatronics.easyscanner

import com.rawmechatronics.easyscanner.CalibrationStatus.Action
import com.rawmechatronics.easyscanner.FrameResult.Calib
import com.rawmechatronics.easyscanner.FrameResult.H
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class CalibrationStatusTest {

    private fun status(state: Int, count: Int = 0, missing: Int = 0, exhausted: Boolean = false, rejection: Int = 0) =
        CalibrationStatus.of(
            FrameResult(H.LEN).apply {
                data.fill(Float.NaN)
                data[H.CALIB_STATE] = state.toFloat()
                data[H.CALIB_COUNT] = count.toFloat()
                data[H.CALIB_TARGET] = 64f
                data[H.CALIB_REJECTION] = rejection.toFloat()
                data[H.CALIB_MISSING_SIDES] = missing.toFloat()
                data[H.CALIB_EXHAUSTED] = if (exhausted) 1f else 0f
                length = H.LEN
            }
        )

    @Test
    fun theMainButtonFollowsTheState() {
        assertEquals(Action.START, status(Calib.IDLE).action)
        assertEquals(Action.PAUSE, status(Calib.COLLECTING, 10).action)
        assertEquals(Action.WAIT, status(Calib.SOLVING, 64).action)
        assertEquals(Action.RESUME, status(Calib.PAUSED, 10).action)
        assertEquals(Action.START_AGAIN, status(Calib.SAVED).action)
    }

    @Test
    fun aRejectionAddsViewsUntilTheSetIsFull() {
        val rejected = status(Calib.REJECTED, 64, rejection = Calib.REJECT_FOCAL_UNCERTAIN)
        assertEquals(Action.ADD_VIEWS, rejected.action)
        assertEquals(Calib.REJECT_FOCAL_UNCERTAIN, rejected.rejection)
        assertEquals(Action.START_OVER, status(Calib.REJECTED, 128, exhausted = true).action)
    }

    @Test
    fun tiltedViewsAreAskedForOnlyOnceTheTargetIsReached() {
        assertFalse(status(Calib.COLLECTING, 30, missing = Calib.SIDE_LEFT).needsTilt)
        assertTrue(status(Calib.COLLECTING, 64, missing = Calib.SIDE_LEFT or Calib.SIDE_ABOVE).needsTilt)
        assertFalse(status(Calib.COLLECTING, 64, missing = 0).needsTilt)
        assertFalse(status(Calib.PAUSED, 64, missing = Calib.SIDE_LEFT).needsTilt)
    }

    @Test
    fun theFrameSizeHoldsWhileViewsWaitForASolve() {
        assertTrue(status(Calib.PAUSED, 12).holdsViews)
        assertTrue(status(Calib.REJECTED, 64).holdsViews)
        assertFalse(status(Calib.SAVED, 0).holdsViews)
        assertFalse(status(Calib.COLLECTING, 0).holdsViews)
    }
}
