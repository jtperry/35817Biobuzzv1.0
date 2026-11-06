package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autos.BlueAuto2Tip;

import dev.nextftc.robot.opmode.NextOpMode;

/** SimLoop builds this by its no-arg constructor inside each run's loader. */
public final class BlueAuto2TipSim extends NextOpModeSim {

    @Override
    protected NextOpMode create(Robot robot) {
        return new BlueAuto2Tip(robot);
    }
}
