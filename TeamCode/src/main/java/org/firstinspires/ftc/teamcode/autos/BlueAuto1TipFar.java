package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Auto 1 Tip Far", preselectTeleop = "Main TeleOp")
public class BlueAuto1TipFar extends MainAuto1TipFar {
    public BlueAuto1TipFar(Robot robot) {
        super(robot, Alliance.BLUE, new AutoRoute1TipFar.Blue());
    }
}
