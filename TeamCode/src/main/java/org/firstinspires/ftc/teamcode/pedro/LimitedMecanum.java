package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Mecanum with a max wheel power for path following. */
public class LimitedMecanum extends Mecanum {

    /** Max wheel power while following, 0-1. Manual drive ignore it. */
    public static double maxPower = 1.0;

    public LimitedMecanum(HardwareMap map, MecanumConfig config) {
        super(map, config);
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        if (!manual && maxPower < 1.0) {
            // Biggest wheel power ends at maxPower, same direction mix.
            double biggest = 1.0;
            for (double wheel : computeWheelPowersUnnormalized(powers)) {
                biggest = Math.max(biggest, Math.abs(wheel));
            }
            double scale = maxPower / biggest;
            powers = new DrivePowers(
                    powers.forward() * scale, powers.strafe() * scale, powers.turn() * scale);
        }
        super.drive(powers, manual);
    }
}
