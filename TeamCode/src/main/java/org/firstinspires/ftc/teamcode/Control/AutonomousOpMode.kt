package org.firstinspires.ftc.teamcode.Control

import org.firstinspires.ftc.robotcore.external.Telemetry

interface AutonomousOpMode {
    val opModeIsActive: Boolean

    val telemetry: Telemetry
}
