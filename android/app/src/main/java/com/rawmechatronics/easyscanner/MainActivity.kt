package com.rawmechatronics.easyscanner

import android.Manifest
import android.content.ContentValues
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.SurfaceTexture
import android.hardware.SensorManager
import android.hardware.camera2.CameraManager
import android.net.Uri
import android.os.Bundle
import android.os.Environment
import android.os.Handler
import android.os.Looper
import android.provider.MediaStore
import android.provider.Settings
import android.util.Log
import android.view.TextureView
import android.view.WindowManager
import androidx.activity.OnBackPressedCallback
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.core.content.ContextCompat
import androidx.preference.PreferenceManager
import com.google.android.material.dialog.MaterialAlertDialogBuilder
import com.google.android.material.snackbar.Snackbar
import com.rawmechatronics.easyscanner.FrameResult.Calib
import com.rawmechatronics.easyscanner.FrameResult.Ev
import com.rawmechatronics.easyscanner.FrameResult.H
import com.rawmechatronics.easyscanner.FrameResult.Photo
import com.rawmechatronics.easyscanner.databinding.ActivityMainBinding
import org.json.JSONObject
import java.io.File
import java.io.IOException
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.roundToInt

/**
 * The scanner screen: live preview + pose overlay + HUD under one of two
 * panels — the guided steps (board → focus → camera → laser plane → scan →
 * saved), or in expert mode the desktop prototype's four modes: viewfinder,
 * camera (intrinsics) calibration, laser-plane calibration and scanning.
 */
class MainActivity : AppCompatActivity(), FrameProcessor.Listener {

    private enum class Mode { VIEW, CAMERA, LASER, SCAN }

    private lateinit var b: ActivityMainBinding
    private lateinit var mgr: CameraManager
    private lateinit var processor: FrameProcessor
    private lateinit var camera: CameraController
    private lateinit var gyro: GyroStream
    private val main = Handler(Looper.getMainLooper())

    @Volatile private var info: CameraInfo? = null
    @Volatile private var governor: ResolutionGovernor? = null
    private var settings: AppSettings? = null
    private var mode = Mode.VIEW
    private var messages = JSONObject()
    private val fps = FpsMeter()
    private val hudPending = AtomicBoolean(false)

    // last header values the buttons depend on (UI thread)
    private var calibActive = false
    private var planeActive = false
    private var scanOn = false
    /** This side asked the scan to stop; any other stop came from the core (cloud full, board moved). */
    private var scanStopRequested = false

    /** The open photo session's directory (set on the UI thread, read on the still thread). */
    /** Back while a scan session is open asks first instead of leaving it behind. */
    private val leaveScanGuard = object : OnBackPressedCallback(false) {
        override fun handleOnBackPressed() = confirmLeaveScan()
    }
    @Volatile private var photoDir: File? = null
        set(value) {
            field = value
            leaveScanGuard.isEnabled = value != null
        }
    /**
     * The serial this side gave its photo session. The core echoes the serial
     * of the last session it opened, so "off" with OUR serial means the core
     * closed it, and results from before the session started are told apart.
     */
    private var photoSerial = 0

    private val wizard = Wizard()
    private var readiness = Readiness(boardConfirmed = false, focusLocked = false, calibrated = false, planeReady = false)
    private lateinit var panel: PanelView
    /** The last copy to Download, for the Saved step, and the session it is of. */
    private var copied: SessionExport.Copied? = null
    private var exporting: File? = null
    private var copyNote = ""
    /** What the copy holds, under [copyNote]. */
    private var copyDetail: String? = null
    /** Unsaved sessions are offered once per process. */
    private var pendingOffered = false

    private val expert get() = settings?.expertMode == true

    // on a grant the activity resumes next, and onResume opens the camera
    private val askCamera = registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
        if (!granted) {
            cameraDenied = true
            explainCameraDenied()
        }
    }

    /**
     * The user turned the camera permission down. The system prompt pauses and
     * resumes the activity, so asking again from onResume would put it right
     * back up; from here on only the explanation with a way to Settings shows.
     */
    private var cameraDenied = false
    private var cameraDeniedDialog: AlertDialog? = null

    /** The camera was (re)opened for the current resume; reset by onPause. */
    private var cameraStarted = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        b = ActivityMainBinding.inflate(layoutInflater)
        setContentView(b.root)
        onBackPressedDispatcher.addCallback(this, leaveScanGuard)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        mgr = getSystemService(CameraManager::class.java)
        processor = FrameProcessor(filesDir.absolutePath, this)
        camera = CameraController(
            mgr, processor.handler, processor,
            onError = { msg -> main.post { onError(msg) } },
            onSizeFailed = { size -> main.post { fallBackFrom(size) } },
            onStill = ::onStill,
            onStillFailed = { msg, fatal -> main.post { onStillFailed(msg, fatal) } },
        )
        gyro = GyroStream(getSystemService(SensorManager::class.java))
        processor.gyro = gyro
        processor.timing = camera
        b.overlay.source = processor.results
        settings = AppSettings.load(this)

        panel = PanelView(b)
        b.settingsButton.setOnClickListener { startActivity(Intent(this, SettingsActivity::class.java)) }
        b.libraryButton.setOnClickListener { startActivity(Intent(this, LibraryActivity::class.java)) }
        b.stepLabel.setOnClickListener { showSteps() }
        b.boardStatus.setOnLongClickListener {
            toggleDebugOverlay()
            true
        }
        b.modes.check(R.id.modeView)
        b.modes.addOnButtonCheckedListener { _, id, checked ->
            if (checked && expert) setMode(
                when (id) {
                    R.id.modeCamera -> Mode.CAMERA
                    R.id.modeLaser -> Mode.LASER
                    R.id.modeScan -> Mode.SCAN
                    else -> Mode.VIEW
                }
            )
        }
        b.texture.surfaceTextureListener = object : TextureView.SurfaceTextureListener {
            override fun onSurfaceTextureAvailable(st: SurfaceTexture, w: Int, h: Int) = startCamera()
            override fun onSurfaceTextureSizeChanged(st: SurfaceTexture, w: Int, h: Int) {}
            override fun onSurfaceTextureDestroyed(st: SurfaceTexture) = true
            override fun onSurfaceTextureUpdated(st: SurfaceTexture) {}
        }
        applyInterface()
        setMode(Mode.VIEW)
    }

    override fun onResume() {
        super.onResume()
        when {
            ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED -> {
                // granted, possibly in Settings after a denial
                cameraDenied = false
                cameraDeniedDialog?.dismiss()
                startCamera()
            }
            cameraDenied -> explainCameraDenied()
            else -> askCamera.launch(Manifest.permission.CAMERA)
        }
    }

    override fun onPause() {
        // the scan session stays open and carries on after the resume (unless the
        // settings change the board or the camera); its cloud goes to disk now,
        // in case the process does not come back
        if (photoDir != null) processor.command(Cmd.CHECKPOINT)
        camera.close()
        gyro.stop()
        cameraStarted = false
        super.onPause()
    }

    override fun onDestroy() {
        camera.quit()
        processor.release()
        super.onDestroy()
    }

    /** (Re)read the settings and (re)open the camera with them. */
    private fun startCamera() {
        if (cameraStarted || !b.texture.isAvailable || !hasCameraPermission()) return
        cameraStarted = true
        val st = AppSettings.load(this)
        settings = st
        applyInterface()
        val ids = CameraInfo.backCameraIds(mgr)
        if (ids.isEmpty()) {
            showNote(getString(R.string.no_camera), Snackbar.LENGTH_INDEFINITE)
            return
        }
        val inf = CameraInfo.query(mgr, st.cameraId.takeIf { it in ids } ?: ids.first(), st.targetFps, st.maxLongSide)
        info = inf
        val pv = inf.previewSize
        val portrait = inf.sensorOrientation % 180 != 0
        (b.previewFrame.layoutParams as ConstraintLayout.LayoutParams).dimensionRatio =
            if (portrait) "${pv.height}:${pv.width}" else "${pv.width}:${pv.height}"
        b.previewFrame.requestLayout()
        b.overlay.sensorOrientation = inf.sensorOrientation

        processor.configure(st.nativeConfig(inf))
        if (inf.gyroClockShared) gyro.start() else gyro.stop()
        val sizes = inf.analysisSizes
        governor = if (st.autoResolution) {
            ResolutionGovernor(sizes.map { FrameSize(it.width, it.height) }, st.targetFps, 0)
                .also { it.reset(0, System.nanoTime()) }
        } else null
        camera.open(inf, b.texture.surfaceTexture!!, sizes.first(), st)
        renderPanel()
        offerPending()
    }

    /** The camera refused [size]: continue one size smaller and never retry it. */
    private fun fallBackFrom(size: android.util.Size) {
        val sizes = info?.analysisSizes ?: return
        val next = sizes.indexOf(size) + 1
        if (next <= 0 || next > sizes.lastIndex) {
            onError("camera session failed at ${size.width}x${size.height}")
            return
        }
        showNote("camera refused ${size.width}x${size.height}, using ${sizes[next].width}x${sizes[next].height}")
        processor.handler.post { governor?.limitTo(next, System.nanoTime()) }
        camera.setAnalysisSize(sizes[next])
    }

    private fun hasCameraPermission() =
        ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED

    /** Settings changed here (focus lock, board, laser): reload them and hand them on. */
    private fun applySettingsLive() {
        val st = AppSettings.load(this)
        settings = st
        camera.applySettings(st)
        processor.configure(st.nativeConfig(info))
    }

    // ── processing-thread callbacks ──────────────────────────────────────────

    override fun onResult(r: FrameResult) {
        fps.tick(r.timestampNs)
        governor?.let { g ->
            val photos = r[H.PHOTO_STATE].toInt()
            // one size per calibration solve, and per photo session (its stills share one K)
            g.frozen = r.flag(H.CALIB_ACTIVE) || r[H.CALIB_STATE].toInt() == Calib.SOLVING ||
                (photos != Photo.OFF && photos != Photo.NO_PHOTOS)
            val from = g.size
            // frames still in flight from before a switch say nothing about the new size
            if (r[H.FRAME_W].toInt() != from.width || r[H.FRAME_H].toInt() != from.height) return@let
            g.onFrame(r.procMs, System.nanoTime())?.let { idx ->
                info?.analysisSizes?.getOrNull(idx)?.let { size ->
                    Log.i(TAG, "resolution $from -> ${size.width}x${size.height}")
                    camera.setAnalysisSize(size)
                }
            }
        }
        if (r.events() and (Ev.INTRINSICS_SAVED or Ev.PLANE_SAVED) != 0) {
            // a re-done calibration step is complete: the guided flow moves on
            main.post { wizard.completed(currentReadiness()) }
        }
        b.overlay.postInvalidateOnAnimation()
        if (hudPending.compareAndSet(false, true)) {
            main.postDelayed({
                hudPending.set(false)
                renderHud()
            }, HUD_PERIOD_MS)
        }
    }

    override fun onPhotoDue() {
        camera.captureStill()
    }

    /** A still is back (still thread): onto disk, then to the core to be posed and named. */
    private fun onStill(s: Still) {
        val file = photoDir?.let { File(it, "photos/.pending_${s.timestampNs}.jpg") } ?: return
        try {
            file.writeBytes(s.jpeg)
        } catch (e: IOException) {
            main.post { onStillFailed("photo not written: ${e.message}", false) }
            return
        }
        val arg = JSONObject().put("file", file.absolutePath).put("ts", s.timestampNs)
            .put("w", s.width).put("h", s.height)
        processor.command(Cmd.PHOTO_ADD, arg.toString())
    }

    /**
     * A still did not happen: the core backs off (and stops the photos after a
     * few); when none is possible at all the session carries on without photos.
     */
    private fun onStillFailed(msg: String, fatal: Boolean) {
        if (photoDir == null) return
        showNote(msg)
        if (fatal) {
            processor.command(Cmd.PHOTOS_OFF, msg) { renderPanel() }
            camera.setStills(emptyList())
        } else {
            processor.command(Cmd.PHOTO_FAILED, msg) { renderPanel() }
        }
    }

    override fun onMessages(json: String) {
        messages = runCatching { JSONObject(json) }.getOrDefault(messages)
        renderPanel()
    }

    override fun onError(msg: String) = showNote(msg)

    // ── preview overlays and the panel ───────────────────────────────────────

    private fun renderHud() {
        val r = processor.results.latest() ?: return
        val st = settings
        if (st?.debugOverlay == true) {
            b.hud.text = DebugHud.text(
                r,
                DebugHud.Context(
                    fps = fps.fps,
                    resolution = governor?.status,
                    intrinsics = messages.optString("intrinsics", "?"),
                    messages = coreMessages(),
                ),
            )
        }
        panel.renderBoardStatus(boardStatusOf(r))
        panel.renderCoach(st?.let { stallHint(r, it) })

        // the core closed our session (board, camera or calibration changed, the laser plane re-fitted)
        val closedByCore = r[H.PHOTO_STATE].toInt() == Photo.OFF && photoDir != null &&
            r[H.PHOTO_SERIAL].toInt() == photoSerial
        // every read of r is above: what follows can render, which takes a newer frame
        val newCalib = r.flag(H.CALIB_ACTIVE)
        val newPlane = r.flag(H.PLANE_ACTIVE)
        val newScan = r.flag(H.SCAN_ON)
        if (closedByCore) {
            val dir = photoDir
            endPhotos()
            // its files are kept: hand them to Download right away rather than at the next start
            dir?.let { exportClosed(it) }
        }
        if (newCalib != calibActive || newPlane != planeActive || newScan != scanOn) {
            if (scanOn && !newScan && !scanStopRequested && !closedByCore) explainScanStop()
            if (!newScan) scanStopRequested = false
            calibActive = newCalib
            planeActive = newPlane
            scanOn = newScan
            // nothing to reach for in the top corner while walking around the object
            b.topActions.showIf(!scanOn)
        }
        renderPanel()
    }

    /** The core's latest messages, for the debug overlay and expert mode. */
    private fun coreMessages(): List<String> =
        messages.keys().asSequence().map { messages.optString(it, "") }.filter { it.isNotEmpty() }.toList()

    private fun boardStatusOf(r: FrameResult) = when {
        r.flag(H.DETECTED) -> BoardStatus(getString(R.string.board_status_found), BoardStatus.Kind.FOUND)
        r[H.N_MARKERS] >= MISMATCH_MARKERS -> BoardStatus(getString(R.string.board_status_mismatch), BoardStatus.Kind.MISMATCH)
        else -> BoardStatus(getString(R.string.board_status_none), BoardStatus.Kind.NONE)
    }

    /**
     * Why the running step makes no progress on the latest frame, in plain
     * words: the counters alone do not say that the board is out of view or
     * that the laser colour setting does not match the laser. Null while the
     * step is idle or progressing.
     */
    private fun stallHint(r: FrameResult, st: AppSettings): String? {
        val running = when (mode) {
            Mode.CAMERA -> calibActive
            Mode.LASER -> planeActive
            Mode.SCAN -> scanOn
            Mode.VIEW -> false
        }
        if (!running) return null
        if (!r.flag(H.DETECTED)) return getString(R.string.stall_no_board)
        if (mode == Mode.CAMERA) return null
        when (r[H.LASER_STATE].toInt()) {
            FrameResult.Laser.OFF -> return getString(R.string.stall_laser_off)
            FrameResult.Laser.NONE -> return getString(R.string.stall_no_laser, laserColorLabel(st.laserColor))
        }
        val gate = r[H.SCAN_GATE]
        if (mode == Mode.SCAN && !gate.isNaN() && gate.toInt() != FrameResult.Gate.KEPT) {
            return getString(R.string.stall_gate, FrameResult.Gate.label(gate.toInt()))
        }
        return null
    }

    /** The label Settings shows for a laser colour value. */
    private fun laserColorLabel(value: String): String {
        val i = resources.getStringArray(R.array.laser_color_values).indexOf(value)
        return resources.getStringArray(R.array.laser_color_labels).getOrElse(i) { value }
    }

    /** The photo keyframe state as the scanning user needs it. */
    private fun photoStateText(state: Int) = getString(
        when (state) {
            Photo.IN_FLIGHT -> R.string.photo_taking
            Photo.HOLD_NO_BOARD, Photo.HOLD_FEW_CORNERS -> R.string.photo_hold_board
            Photo.HOLD_POSE_RMS, Photo.HOLD_SETTLING, Photo.HOLD_MOVING -> R.string.photo_hold_steady
            else -> R.string.photo_waiting
        }
    )

    /** Settings › Debug overlay, also toggled by a long press on the board status. */
    private fun toggleDebugOverlay() {
        val on = settings?.debugOverlay != true
        prefs().edit().putBoolean(AppSettings.KEY_DEBUG_OVERLAY, on).apply()
        settings = AppSettings.load(this)
        applyInterface()
        showNote(getString(if (on) R.string.debug_overlay_on else R.string.debug_overlay_off), Snackbar.LENGTH_SHORT)
    }

    /**
     * [keepSession]: leaving Scan only pauses the scan and saves its cloud (the
     * guided flow comes back to the same session); otherwise the session ends.
     */
    private fun setMode(m: Mode, keepSession: Boolean = false) {
        // leaving a mode stops what it was doing
        when (mode) {
            Mode.CAMERA -> if (m != Mode.CAMERA && calibActive) processor.command(Cmd.CALIB_STOP)
            Mode.LASER -> if (m != Mode.LASER && planeActive) processor.command(Cmd.PLANE_STOP)
            Mode.SCAN -> if (m != Mode.SCAN) {
                if (scanOn) stopScan()
                if (!keepSession && photoDir != null) stopPhotos()
            }
            Mode.VIEW -> {}
        }
        mode = m
        renderPanel()
    }

    private fun laserEnabled() = prefs().getBoolean("laser_enabled", true)

    /** Guided steps or the expert mode toggle, and the debug overlay, as the settings say. */
    private fun applyInterface() {
        val e = expert
        b.modes.showIf(e)
        b.hud.showIf(settings?.debugOverlay == true)
        if (e) {
            b.modes.check(
                when (mode) {
                    Mode.VIEW -> R.id.modeView
                    Mode.CAMERA -> R.id.modeCamera
                    Mode.LASER -> R.id.modeLaser
                    Mode.SCAN -> R.id.modeScan
                }
            )
        }
        renderPanel()
    }

    /** The bottom panel: the guided step, or the expert mode's tools. */
    private fun renderPanel() {
        if (expert) panel.render(expertPanel()) else renderWizard()
    }

    /** A short note at the bottom of the screen, above the actions. */
    private fun showNote(text: String, duration: Int = Snackbar.LENGTH_LONG) {
        Snackbar.make(b.root, text, duration)
            .setAnchorView(b.primaryAction)
            .setTextMaxLines(SNACKBAR_LINES)
            .show()
    }

    private fun startPauseAction(running: Boolean, idle: Int = R.string.act_start, run: () -> Unit) = PanelAction(
        getString(if (running) R.string.act_pause else idle),
        icon = if (running) R.drawable.ic_pause else R.drawable.ic_play_arrow,
        run = run,
    )

    // ── expert mode ──────────────────────────────────────────────────────────

    private fun expertPanel(): PanelContent {
        val show: (String) -> Unit = { renderPanel() }
        val details = coreMessages().joinToString("\n").ifEmpty { null }
        return when (mode) {
            Mode.VIEW -> PanelContent(
                instruction = getString(R.string.hint_view),
                detail = details,
                primary = PanelAction(getString(if (laserEnabled()) R.string.act_laser_on else R.string.act_laser_off)) {
                    prefs().edit().putBoolean("laser_enabled", !laserEnabled()).apply()
                    applySettingsLive()
                    renderPanel()
                },
            )
            Mode.CAMERA -> PanelContent(
                instruction = getString(R.string.hint_camera),
                detail = details,
                primary = startPauseAction(calibActive) {
                    processor.command(if (calibActive) Cmd.CALIB_STOP else Cmd.CALIB_START, done = show)
                },
                secondary = PanelAction(getString(R.string.act_capture)) { processor.command(Cmd.CALIB_CAPTURE, done = show) },
                overflow = listOf(
                    PanelAction(getString(R.string.act_reset)) { processor.command(Cmd.CALIB_RESET, done = show) },
                    PanelAction(getString(R.string.act_forget_k)) { processor.command(Cmd.FORGET_INTRINSICS, done = show) },
                ),
            )
            Mode.LASER -> PanelContent(
                instruction = getString(R.string.hint_laser),
                detail = details,
                primary = startPauseAction(planeActive) {
                    processor.command(if (planeActive) Cmd.PLANE_STOP else Cmd.PLANE_START, done = show)
                },
                secondary = PanelAction(getString(R.string.act_force)) { processor.command(Cmd.PLANE_FORCE, done = show) },
                overflow = listOf(
                    PanelAction(getString(R.string.act_forget_plane)) { processor.command(Cmd.FORGET_PLANE, done = show) },
                ),
            )
            Mode.SCAN -> PanelContent(
                instruction = getString(R.string.hint_scan),
                help = getString(R.string.help_expert_scan),
                detail = details,
                primary = PanelAction(
                    getString(if (scanOn) R.string.act_stop else R.string.act_start),
                    icon = if (scanOn) R.drawable.ic_pause else R.drawable.ic_play_arrow,
                ) { if (scanOn) stopScan(show) else processor.command(Cmd.SCAN_START, done = show) },
                secondary = PanelAction(getString(R.string.act_export), icon = R.drawable.ic_share) { exportPly() },
                overflow = listOfNotNull(
                    PanelAction(getString(if (photoDir != null) R.string.act_photos_on else R.string.act_photos_off)) {
                        if (photoDir == null) startPhotos() else stopPhotos()
                    },
                    photoDir?.let { PanelAction(getString(R.string.act_photo_now)) { processor.command(Cmd.PHOTO_SNAP, done = show) } },
                    PanelAction(getString(R.string.act_clear)) { processor.command(Cmd.CLOUD_CLEAR, done = show) },
                ),
            )
        }
    }

    /** Write the cloud as a PLY into the app's files, then copy it to Download and offer it. */
    private fun exportPly() {
        val dir = File(getExternalFilesDir(null), "scans").apply { mkdirs() }
        val name = "scan_" + SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date()) + ".ply"
        val file = File(dir, name)
        processor.command(Cmd.EXPORT_PLY, file.absolutePath) { msg ->
            renderPanel()
            showNote(msg)
            if (msg.startsWith("exported")) shareScan(file)   // engine.rs export_ply
        }
    }

    // ── guided steps ─────────────────────────────────────────────────────────

    private fun readinessOf(r: FrameResult): Readiness {
        val st = settings
        return Readiness(
            boardConfirmed = st != null && st.boardConfirmed == st.board.key,
            focusLocked = info?.manualFocus != true || st?.manualFocus == true,
            calibrated = r.flag(H.K_OK),
            planeReady = r.flag(H.PLANE_OK),
        )
    }

    private fun currentReadiness(): Readiness {
        processor.results.latest()?.let { readiness = readinessOf(it) }
        return readiness
    }

    private fun modeFor(s: Step) = when (s) {
        Step.CAMERA -> Mode.CAMERA
        Step.LASER -> Mode.LASER
        Step.SCAN -> Mode.SCAN
        Step.BOARD, Step.FOCUS, Step.DONE -> Mode.VIEW
    }

    private fun openStep(s: Step) {
        if (expert) return
        val rd = currentReadiness()
        if (!wizard.open(s, rd)) {
            showNote(getString(R.string.step_locked, getString(stepLabel(rd.firstOpen))), Snackbar.LENGTH_SHORT)
        }
        renderWizard()
    }

    private fun stepLabel(s: Step) = when (s) {
        Step.BOARD -> R.string.step_board
        Step.FOCUS -> R.string.step_focus
        Step.CAMERA -> R.string.step_camera
        Step.LASER -> R.string.step_laser
        Step.SCAN, Step.DONE -> R.string.step_scan
    }

    /** All steps with their state; a tap opens a finished step again or the first open one. */
    private fun showSteps() {
        val rd = currentReadiness()
        val labels = STEPS.map { s ->
            val name = getString(stepLabel(s))
            when {
                s == wizard.step || wizard.step == Step.DONE && s == Step.SCAN -> getString(R.string.step_entry_current, name)
                rd.done(s) -> getString(R.string.step_entry_done, name)
                else -> getString(R.string.step_entry_todo, name)
            }
        }
        MaterialAlertDialogBuilder(this)
            .setTitle(R.string.steps_title)
            .setItems(labels.toTypedArray()) { _, i -> openStep(STEPS[i]) }
            .setNegativeButton(android.R.string.cancel, null)
            .show()
    }

    /** The guided panel for the step the wizard is on. */
    private fun renderWizard() {
        if (expert) return
        val r = processor.results.latest()
        val rd = r?.let { readinessOf(it) } ?: readiness
        readiness = rd
        val step = wizard.update(rd)
        if (modeFor(step) != mode) {
            // setMode renders the panel itself, from a newer frame than r
            setMode(modeFor(step), keepSession = true)
            return
        }
        val st = settings ?: return
        val position = STEPS.indexOf(if (step == Step.DONE) Step.SCAN else step)
        panel.setStepPosition(position, STEPS.size)
        val content = guidedPanel(step, r, st).copy(
            stepLabel = if (step == Step.DONE) null else getString(R.string.step_position, position + 1, STEPS.size),
        )
        panel.render(content)
    }

    private fun guidedPanel(step: Step, r: FrameResult?, st: AppSettings): PanelContent {
        fun f(slot: Int) = r?.get(slot) ?: Float.NaN
        return when (step) {
            Step.BOARD -> {
                val board = st.board
                val name = AppSettings.BOARD_PRESETS.firstOrNull { it.board == board }?.label
                    ?: "${board.squaresX}×${board.squaresY} · ${board.squareMm} / ${board.markerMm} mm · dict ${board.dict}"
                val seen = r?.flag(H.DETECTED) == true
                val mismatch = !seen && f(H.N_MARKERS) >= MISMATCH_MARKERS
                PanelContent(
                    title = getString(R.string.title_board),
                    instruction = getString(R.string.instr_board),
                    help = getString(R.string.help_board),
                    detail = getString(R.string.board_detail, name),
                    notice = if (mismatch) Notice(getString(R.string.board_mismatch), Notice.Level.WARNING) else null,
                    primary = PanelAction(
                        getString(R.string.act_board_ok),
                        enabled = seen && f(H.N_CORNERS) >= MIN_BOARD_CORNERS,
                        icon = R.drawable.ic_check,
                    ) { confirmBoard() },
                    secondary = PanelAction(getString(R.string.act_board_change)) { chooseBoard() },
                )
            }
            Step.FOCUS -> {
                val d = camera.focusDiopters
                val kFocus = messages.optDouble("k_focus").toFloat()
                val progress = when {
                    st.manualFocus -> Progress(1f, getString(R.string.focus_locked, focusText(st.focusDiopters)))
                    d != null && camera.focusSettled -> Progress(1f, getString(R.string.focus_settled, focusText(d)))
                    else -> Progress(null, getString(R.string.focus_moving))
                }
                PanelContent(
                    title = getString(R.string.title_focus),
                    instruction = getString(R.string.instr_focus),
                    help = getString(R.string.help_focus),
                    progress = progress,
                    detail = if (kFocus.isNaN()) null else getString(R.string.focus_calibrated, focusText(kFocus)),
                    // not only once settled: some autofocus never reports it on a flat board
                    primary = PanelAction(getString(R.string.act_focus_lock), enabled = d != null && !st.manualFocus) {
                        d?.let { lockFocus(it) }
                    },
                    secondary = when {
                        st.manualFocus -> PanelAction(getString(R.string.act_focus_auto)) { unlockFocus() }
                        !kFocus.isNaN() -> PanelAction(getString(R.string.act_focus_calibrated)) { lockFocus(kFocus) }
                        else -> null
                    },
                )
            }
            Step.CAMERA -> PanelContent(
                title = getString(R.string.title_camera),
                instruction = getString(R.string.instr_camera),
                help = getString(R.string.help_camera),
                progress = countProgress(f(H.CALIB_COUNT), f(H.CALIB_TARGET), R.string.camera_progress),
                primary = startPauseAction(calibActive) {
                    processor.command(if (calibActive) Cmd.CALIB_STOP else Cmd.CALIB_START)
                },
                overflow = listOf(PanelAction(getString(R.string.act_reset)) { processor.command(Cmd.CALIB_RESET) }),
            )
            Step.LASER -> {
                val stale = r?.flag(H.PLANE_STALE) == true && !planeActive
                PanelContent(
                    title = getString(R.string.title_laser),
                    instruction = getString(R.string.instr_laser),
                    help = getString(R.string.help_laser),
                    progress = countProgress(f(H.PLANE_LINES), f(H.PLANE_TARGET), R.string.laser_progress),
                    notice = if (stale) Notice(getString(R.string.laser_stale), Notice.Level.WARNING) else null,
                    primary = startPauseAction(planeActive) {
                        if (planeActive) processor.command(Cmd.PLANE_STOP) else {
                            ensureLaserOn()
                            processor.command(Cmd.PLANE_START)
                        }
                    },
                    secondary = if (planeActive) PanelAction(
                        getString(R.string.act_force), enabled = f(H.PLANE_LINES) >= MIN_PLANE_LINES,
                    ) { processor.command(Cmd.PLANE_FORCE) } else null,
                )
            }
            Step.SCAN -> {
                val open = photoDir != null
                val photoState = f(H.PHOTO_STATE).toInt()
                val photos = open && photoState != Photo.NO_PHOTOS
                val instruction = getString(R.string.instr_scan) +
                    if (st.photos.withScan) " " + getString(R.string.instr_scan_photos) else ""
                PanelContent(
                    title = getString(R.string.title_scan),
                    instruction = instruction,
                    help = getString(R.string.help_scan),
                    metric = if (open) getString(R.string.scan_points, pointsText(f(H.CLOUD_POINTS))) else null,
                    detail = if (photos) getString(
                        R.string.scan_photos_state,
                        resources.getQuantityString(R.plurals.photos, f(H.PHOTO_COUNT).toInt(), f(H.PHOTO_COUNT).toInt()),
                        photoStateText(photoState),
                    ) else null,
                    primary = startPauseAction(scanOn, idle = if (open) R.string.act_resume else R.string.act_start) {
                        toggleScan()
                    },
                    secondary = PanelAction(getString(R.string.act_finish), enabled = open, icon = R.drawable.ic_check) {
                        finishScan()
                    },
                    overflow = listOfNotNull(
                        if (photos) PanelAction(getString(R.string.act_photo_now)) { processor.command(Cmd.PHOTO_SNAP) } else null,
                        PanelAction(getString(R.string.act_discard), enabled = open) { discardScan() },
                    ),
                )
            }
            Step.DONE -> {
                val uri = copied?.scanUri
                val newScan = PanelAction(getString(R.string.act_new_scan), icon = R.drawable.ic_add) { newScan() }
                PanelContent(
                    title = getString(R.string.title_done),
                    instruction = copyNote,
                    help = getString(R.string.help_done),
                    detail = copyDetail,
                    primary = uri?.let { PanelAction(getString(R.string.act_share_ply), icon = R.drawable.ic_share) { sharePly(it) } }
                        ?: newScan,
                    secondary = if (uri != null) newScan else null,
                )
            }
        }
    }

    /** "7 of 20 views" with its bar; nothing before the core reports a target. */
    private fun countProgress(done: Float, target: Float, label: Int): Progress? {
        if (done.isNaN() || target.isNaN() || target <= 0f) return null
        return Progress.of(done.toInt(), target.toInt(), getString(label, done.toInt(), target.toInt()))
    }

    /** A lens focus in diopters as the distance it is sharp at. */
    private fun focusText(d: Float) =
        if (d > 0.01f) getString(R.string.focus_distance, 100f / d) else getString(R.string.focus_far)

    private fun pointsText(n: Float) = when {
        n.isNaN() -> "0"
        n >= 1e6f -> "%.1f M".format(n / 1e6f)
        n >= 1e3f -> "%.0f k".format(n / 1e3f)
        else -> n.toInt().toString()
    }

    private fun prefs() = PreferenceManager.getDefaultSharedPreferences(this)

    private fun confirmBoard() {
        val st = settings ?: return
        prefs().edit().putString("board_confirmed", st.board.key).apply()
        settings = AppSettings.load(this)
        wizard.completed(currentReadiness())
    }

    private fun chooseBoard() {
        val presets = AppSettings.BOARD_PRESETS
        val labels = presets.map { it.label } + getString(R.string.board_other)
        MaterialAlertDialogBuilder(this)
            .setTitle(R.string.board_pick)
            .setItems(labels.toTypedArray()) { _, i ->
                if (i < presets.size) {
                    AppSettings.saveBoard(this, presets[i].board)
                    applySettingsLive()
                    renderWizard()
                } else {
                    startActivity(Intent(this, SettingsActivity::class.java))
                }
            }
            .show()
    }

    /** Lock the lens at [diopters] (where autofocus settled, or where the camera was calibrated). */
    private fun lockFocus(diopters: Float) {
        // the Settings slider holds tenths up to 10 D; the lens may stop sooner
        val closest = ((info?.maxDiopters ?: 10f) * 10).toInt().coerceIn(0, 100)
        prefs().edit()
            .putBoolean("manual_focus", true)
            .putInt("focus_tenths", (diopters * 10).roundToInt().coerceIn(0, closest))
            .apply()
        applySettingsLive()
        wizard.completed(currentReadiness())
    }

    private fun unlockFocus() {
        prefs().edit().putBoolean("manual_focus", false).apply()
        applySettingsLive()
    }

    private fun ensureLaserOn() {
        if (laserEnabled()) return
        prefs().edit().putBoolean("laser_enabled", true).apply()
        applySettingsLive()
    }

    private fun stopScan(done: (String) -> Unit = {}) {
        scanStopRequested = true
        processor.command(Cmd.SCAN_STOP, done = done)
    }

    /** The core ended the scan by itself: say why, or the user keeps sweeping for nothing. */
    private fun explainScanStop() {
        val why = listOf("scan", "surface").map { messages.optString(it, "") }.firstOrNull { it.isNotEmpty() }
        showNote(why ?: getString(R.string.scan_stopped))
    }

    /** Start (a new session for the first start), pause or resume the scan. */
    private fun toggleScan() {
        if (scanOn) {
            stopScan()
            return
        }
        ensureLaserOn()
        if (photoDir != null) {
            processor.command(Cmd.SCAN_START)
            return
        }
        val inf = info ?: return
        // a fresh session starts from an empty cloud, unless it holds points no
        // session saved; the scan runs only once the session is there
        startPhotos(settings?.photos?.withScan == true && inf.stillSizes.isNotEmpty(), fresh = true) {
            processor.command(Cmd.SCAN_START)
        }
    }

    /** Close the session (the core stops the scan and writes its clouds) and copy it to Download. */
    private fun finishScan() {
        val dir = photoDir ?: return
        processor.command(Cmd.PHOTOS_STOP) { exportSession(dir) }
        endPhotos()
        copied = null
        exporting = dir
        copyNote = getString(R.string.done_saving)
        copyDetail = null
        wizard.finished()
    }

    /** Why nothing works without the camera, and the way to grant it in Settings. */
    private fun explainCameraDenied() {
        showNote(getString(R.string.need_camera), Snackbar.LENGTH_INDEFINITE)
        if (cameraDeniedDialog?.isShowing == true) return
        cameraDeniedDialog = MaterialAlertDialogBuilder(this)
            .setTitle(R.string.camera_denied_title)
            .setMessage(R.string.camera_denied_body)
            .setPositiveButton(R.string.act_open_settings) { _, _ ->
                startActivity(Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS, Uri.fromParts("package", packageName, null)))
            }
            .setNegativeButton(R.string.act_ask_again) { _, _ -> askCamera.launch(Manifest.permission.CAMERA) }
            .show()
    }

    /** Back during a scan: finish it, discard it, or stay. */
    private fun confirmLeaveScan() {
        MaterialAlertDialogBuilder(this)
            .setTitle(R.string.leave_scan_title)
            .setMessage(R.string.leave_scan_body)
            .setPositiveButton(R.string.act_finish) { _, _ -> finishScan() }
            .setNeutralButton(R.string.act_discard) { _, _ -> discardScan() }
            .setNegativeButton(R.string.act_keep_scanning, null)
            .show()
    }

    private fun discardScan() {
        MaterialAlertDialogBuilder(this)
            .setTitle(R.string.discard_title)
            .setMessage(R.string.discard_body)
            .setPositiveButton(R.string.act_discard) { _, _ ->
                val dir = photoDir
                // cleared first, so closing the session writes no cloud just to delete it
                processor.command(Cmd.CLOUD_CLEAR)
                processor.command(Cmd.PHOTOS_STOP) { dir?.let { Thread { it.deleteRecursively() }.start() } }
                endPhotos()
                renderWizard()
            }
            .setNegativeButton(android.R.string.cancel, null)
            .show()
    }

    private fun newScan() {
        processor.command(Cmd.CLOUD_CLEAR)
        copied = null
        wizard.newScan(currentReadiness())
    }

    private fun exportSession(dir: File) = Thread {
        val res = runCatching { SessionExport.copyToDownloads(this, dir) }
        runOnUiThread {
            // a later Finish owns the Saved step now
            if (isFinishing || isDestroyed || exporting != dir) return@runOnUiThread
            copied = res.getOrNull()
            copyDetail = res.getOrNull()?.let { contentsText(it) }
            copyNote = res.fold(
                { getString(R.string.done_saved, it.relativeDir) },
                // the core leaves no session behind when nothing was captured
                { if (dir.exists()) getString(R.string.done_failed, it.message, dir.absolutePath) else getString(R.string.done_empty) },
            )
            renderWizard()
        }
    }.start()

    /** What a copied session holds: "scan.ply, laser.ply, 12 photos, … (17 files)". */
    private fun contentsText(c: SessionExport.Copied): String {
        val parts = listOfNotNull(
            SessionExport.SCAN_PLY.takeIf { it in c.names },
            "laser.ply".takeIf { it in c.names },
            if (c.photos > 0) resources.getQuantityString(R.plurals.photos, c.photos, c.photos) else null,
            "session.json".takeIf { it in c.names },
            "recording.jsonl".takeIf { it in c.names },
        )
        return getString(R.string.done_contents, parts.joinToString(", "), c.files)
    }

    /** The core closed the session mid-scan; its files go to Download now, with a note. */
    private fun exportClosed(dir: File) = Thread {
        if (!dir.exists()) return@Thread
        val res = runCatching { SessionExport.copyToDownloads(this, dir) }
        runOnUiThread {
            if (isFinishing || isDestroyed) return@runOnUiThread
            val why = messages.optString("photos", "")
            val note = res.fold({ getString(R.string.closed_saved, it.relativeDir) }, { getString(R.string.closed_kept, dir.name) })
            showNote(if (why.isEmpty()) note else "$note\n$why")
        }
    }.start()

    /** Sessions that never reached Download (the app was killed, or a scan from before): copy them on a yes. */
    private fun offerPending() {
        if (pendingOffered) return
        pendingOffered = true
        val root = File(getExternalFilesDir(null), "sessions")
        val open = photoDir
        Thread {
            val pending = SessionExport.pending(root, open)
            if (pending.isEmpty()) return@Thread
            val latest = pending.last()
            runOnUiThread {
                if (isFinishing || isDestroyed) return@runOnUiThread
                MaterialAlertDialogBuilder(this)
                    .setTitle(R.string.pending_title)
                    .setMessage(getString(R.string.pending_body, pending.size, latest.name, SessionExport.photoCount(latest)))
                    .setPositiveButton(R.string.act_save_now) { _, _ -> copyPending(pending) }
                    .setNegativeButton(R.string.act_later, null)
                    .show()
            }
        }.start()
    }

    private fun copyPending(dirs: List<File>) = Thread {
        val ok = dirs.count { runCatching { SessionExport.copyToDownloads(this, it) }.onFailure { e -> Log.w(TAG, "copy ${it.name}", e) }.isSuccess }
        runOnUiThread { showNote(getString(R.string.pending_saved, ok)) }
    }.start()

    private fun sharePly(uri: Uri) {
        val send = Intent(Intent.ACTION_SEND)
            .setType(PLY_MIME)
            .putExtra(Intent.EXTRA_STREAM, uri)
            .addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
        startActivity(Intent.createChooser(send, getString(R.string.share_scan)))
    }

    // ── export and photo sessions ────────────────────────────────────────────

    /**
     * Copy an exported scan to Download/EasyScanner (our own MediaStore entry needs no
     * storage permission) and offer it to other apps; the original stays in the app's files.
     */
    private fun shareScan(file: File) = Thread {
        val values = ContentValues().apply {
            put(MediaStore.Downloads.DISPLAY_NAME, file.name)
            put(MediaStore.Downloads.MIME_TYPE, PLY_MIME)
            put(MediaStore.Downloads.RELATIVE_PATH, Environment.DIRECTORY_DOWNLOADS + "/" + SessionExport.DOWNLOADS_DIR)
            put(MediaStore.Downloads.IS_PENDING, 1)
        }
        var uri: Uri? = null
        try {
            uri = contentResolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values)
                ?: throw IOException("MediaStore refused the entry")
            contentResolver.openOutputStream(uri)!!.use { out -> file.inputStream().use { it.copyTo(out) } }
            contentResolver.update(uri, ContentValues().apply { put(MediaStore.Downloads.IS_PENDING, 0) }, null, null)
        } catch (e: Exception) {
            Log.w(TAG, "copy to Downloads failed", e)
            uri?.let { contentResolver.delete(it, null, null) }
            uri = null
        }
        runOnUiThread {
            if (isFinishing || isDestroyed) return@runOnUiThread
            if (uri == null) {
                showNote(getString(R.string.scan_copy_failed))
                return@runOnUiThread
            }
            showNote(getString(R.string.scan_saved, "${SessionExport.DOWNLOADS_DIR}/${file.name}"))
            sharePly(uri)
        }
    }.start()

    /**
     * Open a photo session (orbiter-native's layout) under the app's files and,
     * with [photos], add the still stream to the camera; without, the session
     * only keeps the scan. [fresh]: a new scan (see `engine.rs` PhotosStartArg);
     * [onOpen] runs once the core has opened it.
     */
    private fun startPhotos(photos: Boolean = true, fresh: Boolean = false, onOpen: () -> Unit = {}) {
        val inf = info ?: return
        if (photos && inf.stillSizes.isEmpty()) {
            onError(getString(R.string.photos_unavailable))
            return
        }
        val now = Date()
        val id = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(now)
        val utc = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss'Z'", Locale.US)
            .apply { timeZone = TimeZone.getTimeZone("UTC") }.format(now)
        val root = File(getExternalFilesDir(null), "sessions")
        // the core refuses an existing directory: a second start within the same second gets a suffix
        val dir = generateSequence(1) { it + 1 }.map { File(root, if (it == 1) id else "$id-$it") }.first { !it.exists() }
        photoDir = dir
        val serial = ++photoSerial
        if (photos) camera.setStills(inf.stillSizes)
        val arg = JSONObject().put("dir", dir.absolutePath).put("session_id", dir.name)
            .put("started_utc", utc).put("serial", serial).put("photos", photos).put("fresh", fresh)
        processor.command(Cmd.PHOTOS_START, arg.toString()) { msg ->
            renderPanel()
            // engine.rs open_photos: anything else is a refusal
            if (msg.startsWith("photos on")) {
                if (photoSerial == serial && photoDir != null) onOpen()
            } else if (photoSerial == serial && photoDir != null) {
                showNote(msg)
                endPhotos()
            }
        }
        renderPanel()
    }

    private fun stopPhotos() {
        processor.command(Cmd.PHOTOS_STOP) { msg ->
            renderPanel()
            showNote(msg)
        }
        endPhotos()
    }

    /** Forget the session on this side and give the camera its still stream back. */
    private fun endPhotos() {
        photoDir = null
        camera.setStills(emptyList())
        renderPanel()
    }

    /** Delivered frame rate from sensor timestamps (smoothed). */
    private class FpsMeter {
        @Volatile var fps = 0.0
            private set
        private var last = 0L

        fun tick(ts: Long) {
            if (last != 0L && ts > last) {
                val inst = 1e9 / (ts - last)
                fps = if (fps == 0.0) inst else fps * 0.9 + inst * 0.1
            }
            last = ts
        }
    }

    private companion object {
        const val TAG = "EasyScanner"
        const val HUD_PERIOD_MS = 100L
        /** No registered PLY type; a generic one keeps MediaStore from renaming the file. */
        const val PLY_MIME = "application/octet-stream"
        val STEPS = listOf(Step.BOARD, Step.FOCUS, Step.CAMERA, Step.LASER, Step.SCAN)
        /** ArUco markers in view without a board: the settings name another board. */
        const val MISMATCH_MARKERS = 4f
        /** Board corners needed before the guided flow trusts the board. */
        const val MIN_BOARD_CORNERS = 12f
        /** Laser lines needed for an early plane fit. */
        const val MIN_PLANE_LINES = 3f
        const val SNACKBAR_LINES = 4
    }
}
