package org.firstinspires.ftc.teamcode.helpers

val Double.deg get() = Math.toRadians(this)
val Int.deg get() = Math.toRadians(this.toDouble())