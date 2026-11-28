package com.rawmechatronics.easyscanner

import androidx.annotation.DrawableRes

/**
 * What the scanner screen's bottom panel shows for the current guided step or
 * expert mode, as plain data: the activity builds it from the latest frame and
 * [PanelView] puts it on screen. Every field but [instruction] and [primary]
 * is optional and hidden when absent.
 */
data class PanelContent(
    /** "Step 3 of 5" above the title; null in expert mode. */
    val stepLabel: String? = null,
    /** The step's name; null in expert mode, where the mode toggle names it. */
    val title: String? = null,
    /** What to do now, in one or two short sentences. */
    val instruction: String,
    /** The long explanation behind the help button. */
    val help: String? = null,
    /** How far the running step has come. */
    val progress: Progress? = null,
    /** A large figure the step is about, e.g. the scan's point count. */
    val metric: String? = null,
    /** Secondary live state under the instruction. */
    val detail: String? = null,
    /** Something that needs the user's attention before going on. */
    val notice: Notice? = null,
    /** The step's main action: always present, so it keeps its place. */
    val primary: PanelAction,
    val secondary: PanelAction? = null,
    /** Rare or destructive actions, behind the overflow button (destructive ones confirm first). */
    val overflow: List<PanelAction> = emptyList(),
)

/** One button of the panel. [run] is called on the UI thread. */
class PanelAction(
    val label: String,
    val enabled: Boolean = true,
    @DrawableRes val icon: Int? = null,
    val run: () -> Unit,
)

/** A determinate bar when [fraction] is known (0..1), an indeterminate one otherwise. */
data class Progress(val fraction: Float?, val label: String) {
    companion object {
        /** [done] of [target], clamped to a full bar; indeterminate without a target. */
        fun of(done: Int, target: Int, label: String) =
            Progress(if (target > 0) (done.toFloat() / target).coerceIn(0f, 1f) else null, label)
    }
}

data class Notice(val text: String, val level: Level) {
    enum class Level { INFO, WARNING }
}

/** The pill over the top-left corner of the camera picture: is the board in view? */
data class BoardStatus(val label: String, val kind: Kind) {
    enum class Kind { FOUND, MISMATCH, NONE }
}
