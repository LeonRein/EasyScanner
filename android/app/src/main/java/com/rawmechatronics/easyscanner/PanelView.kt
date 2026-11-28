package com.rawmechatronics.easyscanner

import android.content.res.ColorStateList
import android.view.View
import android.widget.TextView
import androidx.appcompat.content.res.AppCompatResources
import androidx.appcompat.widget.PopupMenu
import androidx.core.content.ContextCompat
import com.google.android.material.button.MaterialButton
import com.google.android.material.color.MaterialColors
import com.google.android.material.dialog.MaterialAlertDialogBuilder
import com.rawmechatronics.easyscanner.databinding.ActivityMainBinding
import kotlin.math.roundToInt

/**
 * Puts a [PanelContent] on the scanner screen's bottom panel and the board
 * status onto the preview. It is rendered every few frames, so every setter
 * skips unchanged values: no relayout, and no repeated TalkBack announcements.
 */
class PanelView(private val b: ActivityMainBinding) {

    private val ctx = b.root.context
    private var content: PanelContent? = null

    init {
        b.primaryAction.setOnClickListener { content?.primary?.takeIf { it.enabled }?.run?.invoke() }
        b.secondaryAction.setOnClickListener { content?.secondary?.takeIf { it.enabled }?.run?.invoke() }
        b.overflowButton.setOnClickListener { showOverflow(it) }
        b.helpButton.setOnClickListener { showHelp() }
    }

    fun render(c: PanelContent) {
        content = c
        b.stepHeader.showIf(c.stepLabel != null || c.help != null)
        b.stepLabel.showIf(c.stepLabel != null)
        b.stepProgress.showIf(c.stepLabel != null)
        c.stepLabel?.let { b.stepLabel.setIfChanged(it) }
        b.helpButton.showIf(c.help != null)
        b.stepTitle.showIf(c.title != null)
        c.title?.let { b.stepTitle.setIfChanged(it) }

        b.instruction.setIfChanged(c.instruction)
        b.metric.showText(c.metric)
        b.detail.showText(c.detail)
        renderProgress(c.progress)
        renderNotice(c.notice)

        b.primaryAction.bind(c.primary)
        b.secondaryAction.showIf(c.secondary != null)
        c.secondary?.let { b.secondaryAction.bind(it) }
        b.overflowButton.showIf(c.overflow.isNotEmpty())
    }

    /** Which guided step of [count] is on screen, 0-based; the bar fills up to it. */
    fun setStepPosition(index: Int, count: Int) {
        if (b.stepProgress.max != count) b.stepProgress.max = count
        if (b.stepProgress.progress != index + 1) b.stepProgress.setProgressCompat(index + 1, true)
    }

    fun renderBoardStatus(s: BoardStatus) {
        val chip = b.boardStatus
        if (chip.text.toString() != s.label) chip.text = s.label
        val (icon, color) = when (s.kind) {
            BoardStatus.Kind.FOUND -> R.drawable.ic_check_circle to ContextCompat.getColor(ctx, R.color.status_ok)
            BoardStatus.Kind.MISMATCH -> R.drawable.ic_warning to attrColor(com.google.android.material.R.attr.colorError)
            BoardStatus.Kind.NONE -> R.drawable.ic_info to attrColor(com.google.android.material.R.attr.colorOnSurfaceVariant)
        }
        if (chip.tag != s.kind) {
            chip.tag = s.kind
            chip.setChipIconResource(icon)
            chip.chipIconTint = ColorStateList.valueOf(color)
        }
    }

    /** The coaching line over the preview, or none. */
    fun renderCoach(text: String?) = b.coach.showText(text)

    private fun renderProgress(p: Progress?) {
        b.progressRow.showIf(p != null)
        p ?: return
        b.progressLabel.setIfChanged(p.label)
        val bar = b.progress
        val indeterminate = p.fraction == null
        if (bar.isIndeterminate != indeterminate) {
            // a visible indicator only switches mode while hidden
            bar.visibility = View.INVISIBLE
            bar.isIndeterminate = indeterminate
            bar.visibility = View.VISIBLE
        }
        p.fraction?.let {
            val value = (it * bar.max).roundToInt()
            if (bar.progress != value) bar.setProgressCompat(value, true)
        }
    }

    private fun renderNotice(n: Notice?) {
        b.notice.showIf(n != null)
        n ?: return
        b.noticeText.setIfChanged(n.text)
        val (container, onContainer, icon) = when (n.level) {
            Notice.Level.WARNING -> Triple(
                com.google.android.material.R.attr.colorErrorContainer,
                com.google.android.material.R.attr.colorOnErrorContainer,
                R.drawable.ic_warning,
            )
            Notice.Level.INFO -> Triple(
                com.google.android.material.R.attr.colorSecondaryContainer,
                com.google.android.material.R.attr.colorOnSecondaryContainer,
                R.drawable.ic_info,
            )
        }
        if (b.notice.tag != n.level) {
            b.notice.tag = n.level
            b.notice.setCardBackgroundColor(attrColor(container))
            val fg = attrColor(onContainer)
            b.noticeText.setTextColor(fg)
            b.noticeText.setCompoundDrawablesRelativeWithIntrinsicBounds(icon, 0, 0, 0)
            b.noticeText.compoundDrawableTintList = ColorStateList.valueOf(fg)
        }
    }

    private fun showOverflow(anchor: View) {
        val actions = content?.overflow ?: return
        val menu = PopupMenu(ctx, anchor)
        actions.forEachIndexed { i, a -> menu.menu.add(0, i, i, a.label).isEnabled = a.enabled }
        menu.setOnMenuItemClickListener { item ->
            // the panel may have moved on while the menu was open
            content?.overflow?.getOrNull(item.itemId)?.takeIf { it.enabled }?.run?.invoke()
            true
        }
        menu.show()
    }

    private fun showHelp() {
        val c = content ?: return
        MaterialAlertDialogBuilder(ctx)
            .setTitle(c.title ?: ctx.getString(R.string.help))
            .setMessage(c.help)
            .setPositiveButton(android.R.string.ok, null)
            .show()
    }

    private fun MaterialButton.bind(a: PanelAction) {
        setIfChanged(a.label)
        isEnabled = a.enabled
        if (tag != a.icon) {
            tag = a.icon
            icon = a.icon?.let { AppCompatResources.getDrawable(ctx, it) }
        }
    }

    private fun TextView.showText(s: String?) {
        showIf(s != null)
        s?.let { setIfChanged(it) }
    }

    private fun attrColor(attr: Int) = MaterialColors.getColor(b.root, attr)
}

/** Shows or hides (GONE) the view, touching it only on a change. */
internal fun View.showIf(visible: Boolean) {
    val v = if (visible) View.VISIBLE else View.GONE
    if (visibility != v) visibility = v
}

internal fun TextView.setIfChanged(s: CharSequence) {
    if (text.toString() != s.toString()) text = s
}
