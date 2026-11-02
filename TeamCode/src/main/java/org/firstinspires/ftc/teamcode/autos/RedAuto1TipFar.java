package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Auto 1 Tip Far", preselectTeleop = "Main TeleOp")
public class RedAuto1TipFar extends MainAuto1TipFar {
    public RedAuto1TipFar(Robot robot) {
        super(robot, Alliance.RED, new AutoRoute1TipFar.Red());
    }
}
