package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Auto 2 Tip", preselectTeleop = "Main TeleOp")
public class BlueAuto2Tip extends MainAuto2Tip {
    public BlueAuto2Tip(Robot robot) {
        super(robot, Alliance.BLUE, new AutoRoute.Blue());
    }
}
