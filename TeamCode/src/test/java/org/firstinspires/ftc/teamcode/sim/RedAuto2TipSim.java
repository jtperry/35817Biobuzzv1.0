package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autos.RedAuto2Tip;

import dev.nextftc.robot.opmode.NextOpMode;

/** SimLoop builds this by its no-arg constructor inside each run's loader. */
public final class RedAuto2TipSim extends NextOpModeSim {

    @Override
    protected NextOpMode create(Robot robot) {
        return new RedAuto2Tip(robot);
    }
}
