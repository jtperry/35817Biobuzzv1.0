package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Auto 1 Tip", preselectTeleop = "Main TeleOp")
public class BlueAuto1Tip extends MainAuto1Tip {
    public BlueAuto1Tip(Robot robot) {
        super(robot, Alliance.BLUE, new AutoRoute1Tip.Blue());
    }
}
