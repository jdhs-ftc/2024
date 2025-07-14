package org.firstinspires.ftc.teamcode.helpers

enum class Color(val gbLightPos: Double) {
    NONE(0.0), // OFF
    RED(0.281),
    ORANGE(0.333),
    YELLOW(0.388),
    SAGE(0.444),
    GREEN(0.5),
    AZURE(0.555),
    BLUE(0.611),
    INDIGO(0.666),
    VIOLET(0.722),
    WHITE(1.0);

    companion object {
        fun inverse(pos: Double): Color =
            Color.entries.firstOrNull { it.gbLightPos == pos } ?: Color.NONE
    }
}

