package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Auto 2 Tip", preselectTeleop = "Main TeleOp")
public class RedAuto2Tip extends MainAuto2Tip {
    public RedAuto2Tip(Robot robot) {
        super(robot, Alliance.RED, new AutoRoute.Red());
    }
}
