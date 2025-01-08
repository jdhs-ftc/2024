package org.firstinspires.ftc.teamcode.helpers

import android.annotation.SuppressLint
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.util.Stack

class TelemetryMenu(val telemetry: Telemetry, private val root: MenuElement) {
    private var currentLevel = root

    private var dpadUpPrev = false
    private var dpadDnPrev = false
    private var dpadRightPrev = false
    private var dpadLeftPrev = false
    private var xPrev = false
    private var lbPrev = false

    private var selectedIdx = 0
    private val selectedIdxStack = Stack<Int>()

    /**
     * TelemetryMenu constructor
     * @param telemetry pass in 'telemetry' from your OpMode
     * @param root the root menu element
     */
    init {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        telemetry.msTransmissionInterval = 50
    }

    /**
     * Call this from inside your loop to put the current menu state into
     * telemetry, and process gamepad inputs for navigating the menu
     * @param gamepad the gamepad you want to use to navigate the menu
     */
    fun loop(gamepad: Gamepad) {
        // Capture current state of the gamepad buttons we care about;
        // We can only look once or we risk a race condition
        val dpadUp = gamepad.dpad_up
        val dpadDn = gamepad.dpad_down
        val dpadRight = gamepad.dpad_right
        val dpadLeft = gamepad.dpad_left
        val x = gamepad.x
        val lb = gamepad.left_bumper

        // Figure out who our children our at this level
        // and figure out which item is currently highlighted
        // with the selection pointer
        val children = currentLevel.children()
        val currentSelection: Element = children[selectedIdx]

        // Left and right are inputs to the selected item (if it's an Option)
        if (currentSelection is OptionElement) {
            if (dpadRight && !dpadRightPrev)  // rising edge
            {
                currentSelection.onRightInput()
            } else if (dpadLeft && !dpadLeftPrev)  // rising edge
            {
                currentSelection.onLeftInput()
            }
        }

        // Up and down navigate the current selection pointer
        if (dpadUp && !dpadUpPrev)  // rising edge
        {
            selectedIdx-- // Move selection pointer up
        } else if (dpadDn && !dpadDnPrev)  // rising edge
        {
            selectedIdx++ // Move selection pointer down
        }

        // Make selected index sane (don't let it go out of bounds) :eyes:
        if (selectedIdx >= children.size) {
            selectedIdx = children.size - 1
        } else if (selectedIdx < 0) {
            selectedIdx = 0
        } else if (x && !xPrev)  // rising edge
        {
            // Select up element
            if (currentSelection is SpecialUpElement) {
                // We can only go up if we're not at the root level
                if (currentLevel !== root) {
                    // Restore selection pointer to where it was before
                    selectedIdx = selectedIdxStack.pop()!!

                    // Change to the parent level
                    currentLevel = currentLevel.parent!!
                }
            } else if (currentSelection is OptionElement) {
                currentSelection.onClick()
            } else if (currentSelection is MenuElement) {
                // Save our current selection pointer so we can restore it
                // later if the user navigates back up a level
                selectedIdxStack.push(selectedIdx)

                // We have no idea what's in the submenu :monkey: so best to
                // just set the selection pointer to the first element
                selectedIdx = 0

                // Now the current level becomes the submenu that the selection
                // pointer was on
                currentLevel = currentSelection
            }
        } else if (lb && !lbPrev) {
            // We can only go up if we're not at the root level
            if (currentLevel !== root) {
                // Restore selection pointer to where it was before
                selectedIdx = selectedIdxStack.pop()!!

                // Change to the parent level
                currentLevel = currentLevel.parent!!
            }
        }

        // Save the current button states so that we can look for
        // the rising edge the next time around the loop :)
        dpadUpPrev = dpadUp
        dpadDnPrev = dpadDn
        dpadRightPrev = dpadRight
        dpadLeftPrev = dpadLeft
        xPrev = x
        lbPrev = lb

        // Start building the text display.
        // First, we add the static directions for gamepad operation
        val builder = StringBuilder()
        builder.append("<font color='#119af5' face=monospace>")
        builder.append("Navigate items.....dpad up/down\n")
            .append("Select.............X or Square\n")
            .append("Edit option........dpad left/right\n")
            .append("Up one level.......left bumper\n")
        builder.append("</font>")
        builder.append("\n")

        // Now actually add the menu options. We start by adding the name of the current menu level.
        builder.append("<font face=monospace>")
        builder.append("Current Menu: ").append(currentLevel.name).append("\n")

        // Now we loop through all the child elements of this level and add them
        for (i in children.indices) {
            // If the selection pointer is at this index, put a green dot in the box :)
            if (selectedIdx == i) {
                builder.append("[<font color=green face=monospace>•</font>] ")
            } else {
                builder.append("[ ] ")
            }

            // Figure out who the selection pointer is pointing at :eyes:
            val e = children[i]

            // If it's pointing at a submenu, indicate that it's a submenu to the user
            // by prefixing "> " to the name.
            if (e is MenuElement) {
                builder.append("> ")
            }

            // Finally, add the element's name
            builder.append(e.displayText)

            // We musn't forget the newline
            builder.append("\n")
        }

        // Don't forget to close the font tag either
        builder.append("</font>")

        // Build the string!!!! :nerd:
        val menu = builder.toString()

        // Add it to telemetry
        telemetry.addLine(menu)
    }

    class MenuElement(val name: String, isRoot: Boolean) : Element() {
        private val children = ArrayList<Element>()

        /**
         * Create a new MenuElement; may either be the root menu, or a submenu (set isRoot accordingly)
         * @param name the name for this menu
         * @param isRoot whether this is a root menu, or a submenu
         */
        init {
            // If it's not the root menu, we add the up one level option as the first element
            if (!isRoot) {
                children.add(SpecialUpElement())
            }
        }

        /**
         * Add a child element to this menu (may either be an Option or another menu)
         * @param child the child element to add
         */
        fun addChild(child: Element) {
            child.parent = this
            children.add(child)
        }

        /**
         * Add multiple child elements to this menu (may either be option, or another menu)
         * @param children the children to add
         */
        fun addChildren(children: Array<Element>) {
            for (e in children) {
                e.parent = this
                this.children.add(e)
            }
        }

        override val displayText = name

        internal fun children(): ArrayList<Element> {
            return children
        }
    }

    abstract class OptionElement : Element() {
        /**
         * Override this to get notified when the element is clicked
         */
        open fun onClick() {}

        /**
         * Override this to get notified when the element gets a "left edit" input
         */
        open fun onLeftInput() {}

        /**
         * Override this to get notified when the element gets a "right edit" input
         */
        open fun onRightInput() {}
    }

    class EnumOption(private var name: String, private var e: Array<Enum<*>>) : OptionElement() {
        private var idx: Int = 0

        constructor(name: String, e: Array<Enum<*>>, def: Enum<*>) : this(name, e) {
            idx = def.ordinal
        }

        override fun onLeftInput() {
            idx++

            if (idx > e.size - 1) {
                idx = 0
            }
        }

        override fun onRightInput() {
            idx--

            if (idx < 0) {
                idx = e.size - 1
            }
        }

        override fun onClick() {
            //onRightInput();
        }

        override val displayText = String.format(
            "%s: <font color='#e37c07' face=monospace>%s</font>",
            name,
            e[idx].name
        )

        val value: Enum<*>
            get() = e[idx]
    }

    @SuppressLint("DefaultLocale")
    class IntegerOption(
        private var name: String,
        private var min: Int,
        private var max: Int,
        var value: Int
    ) : OptionElement() {
        override fun onLeftInput() {
            this.value--

            if (this.value < min) {
                this.value = max
            }
        }

        override fun onRightInput() {
            this.value++

            if (this.value > max) {
                this.value = min
            }
        }

        override fun onClick() {
            //onRightInput();
        }

        override val displayText
            get() = String.format(
            "%s: <font color='#e37c07' face=monospace>%d</font>", name,
            this.value
        )
    }

    internal class BooleanOption(private val name: String, def: Boolean, private var trueText: String = "true", private var falseText: String = "false") : OptionElement() {
        var value: Boolean = true
            private set

        init {
            this.value = def
        }


        override fun onLeftInput() {
            this.value = !this.value
        }

        override fun onRightInput() {
            this.value = !this.value
        }

        override fun onClick() {
            this.value = !this.value
        }

        override val displayText: String
            get() {

                val valStr = if (this.value) trueText else falseText

                return String.format(
                    "%s: <font color='#e37c07' face=monospace>%s</font>",
                    name,
                    valStr
                )
            }
    }

    /**
     *
     */
    class StaticItem(name: String) : OptionElement() {
        override val displayText = name
    }

    abstract class StaticClickableOption(name: String) : OptionElement() {
        abstract override fun onClick()

        override val displayText: String = name
    }

    abstract class Element {
        var parent: MenuElement? = null

        abstract val displayText: String
    }

    private class SpecialUpElement : Element() {
        override val displayText = "<font color='#119af5' face=monospace>.. ↰ Up One Level</font>"
    }
}
