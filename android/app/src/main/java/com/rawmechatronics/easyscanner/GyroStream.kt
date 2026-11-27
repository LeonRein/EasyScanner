package com.rawmechatronics.easyscanner

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread

/**
 * The gyroscope's samples, collected on their own thread and handed to the
 * processing thread in batches ([drain]) before each frame. The core fuses
 * them with the board pose (`scan_algo::fusion`); it calibrates the gyro
 * against the camera itself, so the samples go in raw: `SensorEvent` clock,
 * device axes, rad/s.
 */
class GyroStream(private val sensors: SensorManager) : SensorEventListener {

    /** Samples taken out by one [drain]; the arrays are reused. */
    class Batch {
        val timestampsNs = LongArray(CAPACITY)
        /** x, y, z per sample. */
        val rates = FloatArray(3 * CAPACITY)
        var count = 0
            internal set
    }

    private val sensor: Sensor? = sensors.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private var thread: HandlerThread? = null
    private val lock = Any()
    private val pending = Batch()

    val available: Boolean get() = sensor != null

    /** Start sampling (idempotent). */
    fun start() {
        val s = sensor ?: return
        if (thread != null) return
        val t = HandlerThread("es-gyro").apply { start() }
        thread = t
        sensors.registerListener(this, s, SAMPLING_US, Handler(t.looper))
    }

    /** Stop sampling and drop what was not drained. */
    fun stop() {
        sensors.unregisterListener(this)
        thread?.quitSafely()
        thread = null
        synchronized(lock) { pending.count = 0 }
    }

    /** Move the samples collected since the last call into [into]. */
    fun drain(into: Batch) {
        synchronized(lock) {
            val n = pending.count
            pending.timestampsNs.copyInto(into.timestampsNs, 0, 0, n)
            pending.rates.copyInto(into.rates, 0, 0, 3 * n)
            into.count = n
            pending.count = 0
        }
    }

    override fun onSensorChanged(e: SensorEvent) {
        synchronized(lock) {
            // nobody drained for seconds (no frames): the old samples are useless
            if (pending.count == CAPACITY) pending.count = 0
            val i = pending.count
            pending.timestampsNs[i] = e.timestamp
            pending.rates[3 * i] = e.values[0]
            pending.rates[3 * i + 1] = e.values[1]
            pending.rates[3 * i + 2] = e.values[2]
            pending.count = i + 1
        }
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}

    private companion object {
        /** 200 Hz: the most an app gets without HIGH_SAMPLING_RATE_SENSORS, and plenty for a 30 fps camera. */
        const val SAMPLING_US = 5_000
        /** 2.5 s at 200 Hz. */
        const val CAPACITY = 512
    }
}
