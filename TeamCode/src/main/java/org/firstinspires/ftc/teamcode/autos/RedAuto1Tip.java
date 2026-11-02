package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Auto 1 Tip", preselectTeleop = "Main TeleOp")
public class RedAuto1Tip extends MainAuto1Tip {
    public RedAuto1Tip(Robot robot) {
        super(robot, Alliance.RED, new AutoRoute1Tip.Red());
    }
}
