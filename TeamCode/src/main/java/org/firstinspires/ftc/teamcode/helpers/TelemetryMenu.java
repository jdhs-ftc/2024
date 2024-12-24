package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Stack;

public class TelemetryMenu
    {
        private final MenuElement root;
        private MenuElement currentLevel;

        private boolean dpadUpPrev;
        private boolean dpadDnPrev;
        private boolean dpadRightPrev;
        private boolean dpadLeftPrev;
        private boolean xPrev;
        private boolean lbPrev;

        private int selectedIdx = 0;
        private Stack<Integer> selectedIdxStack = new Stack<>();

        private final Telemetry telemetry;

        /**
         * TelemetryMenu constructor
         * @param telemetry pass in 'telemetry' from your OpMode
         * @param root the root menu element
         */
        public TelemetryMenu(Telemetry telemetry, MenuElement root)
        {
            this.root = root;
            this.currentLevel = root;
            this.telemetry = telemetry;

            telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
            telemetry.setMsTransmissionInterval(50);
        }

        /**
         * Call this from inside your loop to put the current menu state into
         * telemetry, and process gamepad inputs for navigating the menu
         * @param gamepad the gamepad you want to use to navigate the menu
         */
        public void loop(Gamepad gamepad)
        {
            // Capture current state of the gamepad buttons we care about;
            // We can only look once or we risk a race condition
            boolean dpadUp = gamepad.dpad_up;
            boolean dpadDn = gamepad.dpad_down;
            boolean dpadRight = gamepad.dpad_right;
            boolean dpadLeft = gamepad.dpad_left;
            boolean x = gamepad.x;
            boolean lb = gamepad.left_bumper;

            // Figure out who our children our at this level
            // and figure out which item is currently highlighted
            // with the selection pointer
            ArrayList<Element> children = currentLevel.children();
            Element currentSelection = children.get(selectedIdx);

            // Left and right are inputs to the selected item (if it's an Option)
            if (currentSelection instanceof OptionElement)
            {
                if (dpadRight && !dpadRightPrev) // rising edge
                {
                    ((OptionElement) currentSelection).onRightInput();
                }
                else if (dpadLeft && !dpadLeftPrev) // rising edge
                {
                    ((OptionElement) currentSelection).onLeftInput();
                }
            }

            // Up and down navigate the current selection pointer
            if (dpadUp && !dpadUpPrev) // rising edge
            {
                selectedIdx--; // Move selection pointer up
            }
            else if (dpadDn && !dpadDnPrev) // rising edge
            {
                selectedIdx++; // Move selection pointer down
            }

            // Make selected index sane (don't let it go out of bounds) :eyes:
            if (selectedIdx >= children.size())
            {
                selectedIdx = children.size()-1;
            }
            else if (selectedIdx < 0)
            {
                selectedIdx = 0;
            }

            // Select: either enter submenu or input to option
            else if (x && !xPrev) // rising edge
            {
                // Select up element
                if (currentSelection instanceof SpecialUpElement)
                {
                    // We can only go up if we're not at the root level
                    if (currentLevel != root)
                    {
                        // Restore selection pointer to where it was before
                        selectedIdx = selectedIdxStack.pop();

                        // Change to the parent level
                        currentLevel = currentLevel.parent();
                    }
                }
                // Input to option
                else if (currentSelection instanceof OptionElement)
                {
                    ((OptionElement) currentSelection).onClick();
                }
                // Enter submenu
                else if (currentSelection instanceof MenuElement)
                {
                    // Save our current selection pointer so we can restore it
                    // later if the user navigates back up a level
                    selectedIdxStack.push(selectedIdx);

                    // We have no idea what's in the submenu :monkey: so best to
                    // just set the selection pointer to the first element
                    selectedIdx = 0;

                    // Now the current level becomes the submenu that the selection
                    // pointer was on
                    currentLevel = (MenuElement) currentSelection;
                }
            }

            // Go up a level
            else if (lb && !lbPrev)
            {
                // We can only go up if we're not at the root level
                if (currentLevel != root)
                {
                    // Restore selection pointer to where it was before
                    selectedIdx = selectedIdxStack.pop();

                    // Change to the parent level
                    currentLevel = currentLevel.parent();
                }
            }

            // Save the current button states so that we can look for
            // the rising edge the next time around the loop :)
            dpadUpPrev = dpadUp;
            dpadDnPrev = dpadDn;
            dpadRightPrev = dpadRight;
            dpadLeftPrev = dpadLeft;
            xPrev = x;
            lbPrev = lb;

            // Start building the text display.
            // First, we add the static directions for gamepad operation
            StringBuilder builder = new StringBuilder();
            builder.append("<font color='#119af5' face=monospace>");
            builder.append("Navigate items.....dpad up/down\n")
                    .append("Select.............X or Square\n")
                    .append("Edit option........dpad left/right\n")
                    .append("Up one level.......left bumper\n");
            builder.append("</font>");
            builder.append("\n");

            // Now actually add the menu options. We start by adding the name of the current menu level.
            builder.append("<font face=monospace>");
            builder.append("Current Menu: ").append(currentLevel.name).append("\n");

            // Now we loop through all the child elements of this level and add them
            for (int i = 0; i < children.size(); i++)
            {
                // If the selection pointer is at this index, put a green dot in the box :)
                if (selectedIdx == i)
                {
                    builder.append("[<font color=green face=monospace>•</font>] ");
                }
                // Otherwise, just put an empty box
                else
                {
                    builder.append("[ ] ");
                }

                // Figure out who the selection pointer is pointing at :eyes:
                Element e = children.get(i);

                // If it's pointing at a submenu, indicate that it's a submenu to the user
                // by prefixing "> " to the name.
                if (e instanceof MenuElement)
                {
                    builder.append("> ");
                }

                // Finally, add the element's name
                builder.append(e.getDisplayText());

                // We musn't forget the newline
                builder.append("\n");
            }

            // Don't forget to close the font tag either
            builder.append("</font>");

            // Build the string!!!! :nerd:
            String menu = builder.toString();

            // Add it to telemetry
            telemetry.addLine(menu);
        }

        public static class MenuElement extends Element
        {
            private String name;
            private ArrayList<Element> children = new ArrayList<>();

            /**
             * Create a new MenuElement; may either be the root menu, or a submenu (set isRoot accordingly)
             * @param name the name for this menu
             * @param isRoot whether this is a root menu, or a submenu
             */
            public MenuElement(String name, boolean isRoot)
            {
                this.name = name;

                // If it's not the root menu, we add the up one level option as the first element
                if (!isRoot)
                {
                    children.add(new SpecialUpElement());
                }
            }

            /**
             * Add a child element to this menu (may either be an Option or another menu)
             * @param child the child element to add
             */
            public void addChild(Element child)
            {
                child.setParent(this);
                children.add(child);
            }

            /**
             * Add multiple child elements to this menu (may either be option, or another menu)
             * @param children the children to add
             */
            public void addChildren(Element[] children)
            {
                for (Element e : children)
                {
                    e.setParent(this);
                    this.children.add(e);
                }
            }

            @Override
            protected String getDisplayText()
            {
                return name;
            }

            private ArrayList<Element> children()
            {
                return children;
            }
        }

        public static abstract class OptionElement extends Element
        {
            /**
             * Override this to get notified when the element is clicked
             */
            void onClick() {}

            /**
             * Override this to get notified when the element gets a "left edit" input
             */
            protected void onLeftInput() {}

            /**
             * Override this to get notified when the element gets a "right edit" input
             */
            protected void onRightInput() {}
        }

        public static class EnumOption extends OptionElement
        {
            protected int idx = 0;
            protected Enum[] e;
            protected String name;

            public EnumOption(String name, Enum[] e)
            {
                this.e = e;
                this.name = name;
            }

            public EnumOption(String name, Enum[] e, Enum def)
            {
                this(name, e);
                idx = def.ordinal();
            }

            @Override
            public void onLeftInput()
            {
                idx++;

                if(idx > e.length-1)
                {
                    idx = 0;
                }
            }

            @Override
            public void onRightInput()
            {
                idx--;

                if(idx < 0)
                {
                    idx = e.length-1;
                }
            }

            @Override
            public void onClick()
            {
                //onRightInput();
            }

            @Override
            protected String getDisplayText()
            {
                return String.format("%s: <font color='#e37c07' face=monospace>%s</font>", name, e[idx].name());
            }

            public Enum getValue()
            {
                return e[idx];
            }
        }

        public static class IntegerOption extends OptionElement
        {
            protected int i;
            protected int min;
            protected int max;
            protected String name;

            public IntegerOption(String name, int min, int max, int def)
            {
                this.name = name;
                this.min = min;
                this.max = max;
                this.i = def;
            }

            @Override
            public void onLeftInput()
            {
                i--;

                if(i < min)
                {
                    i = max;
                }
            }

            @Override
            public void onRightInput()
            {
                i++;

                if(i > max)
                {
                    i = min;
                }
            }

            @Override
            public void onClick()
            {
                //onRightInput();
            }

            @Override
            protected String getDisplayText()
            {
                return String.format("%s: <font color='#e37c07' face=monospace>%d</font>", name, i);
            }

            public int getValue()
            {
                return i;
            }
        }

        static class BooleanOption extends OptionElement
        {
            private String name;
            private boolean val = true;

            private String customTrue;
            private String customFalse;

            BooleanOption(String name, boolean def)
            {
                this.name = name;
                this.val = def;
            }

            BooleanOption(String name, boolean def, String customTrue, String customFalse)
            {
                this(name, def);
                this.customTrue = customTrue;
                this.customFalse = customFalse;
            }

            @Override
            public void onLeftInput()
            {
                val = !val;
            }

            @Override
            public void onRightInput()
            {
                val = !val;
            }

            @Override
            public void onClick()
            {
                val = !val;
            }

            @Override
            protected String getDisplayText()
            {
                String valStr;

                if(customTrue != null && customFalse != null)
                {
                    valStr = val ? customTrue : customFalse;
                }
                else
                {
                    valStr = val ? "true" : "false";
                }

                return String.format("%s: <font color='#e37c07' face=monospace>%s</font>", name, valStr);
            }

            public boolean getValue()
            {
                return val;
            }
        }

        /**
         *
         */
        public static class StaticItem extends OptionElement
        {
            private String name;

            public StaticItem(String name)
            {
                this.name = name;
            }

            @Override
            protected String getDisplayText()
            {
                return name;
            }
        }

        public static abstract class StaticClickableOption extends OptionElement
        {
            private String name;

            public StaticClickableOption(String name)
            {
                this.name = name;
            }

            abstract void onClick();

            @Override
            protected String getDisplayText()
            {
                return name;
            }
        }

        private static abstract class Element
        {
            private MenuElement parent;

            protected void setParent(MenuElement parent)
            {
                this.parent = parent;
            }

            protected MenuElement parent()
            {
                return parent;
            }

            protected abstract String getDisplayText();
        }

        private static class SpecialUpElement extends Element
        {
            @Override
            protected String getDisplayText()
            {
                return "<font color='#119af5' face=monospace>.. ↰ Up One Level</font>";
            }
        }
    }