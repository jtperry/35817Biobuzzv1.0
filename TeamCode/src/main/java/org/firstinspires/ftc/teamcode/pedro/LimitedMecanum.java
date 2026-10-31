package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Mecanum with max power + max speed for path following. */
public class LimitedMecanum extends Mecanum {

    /** Max wheel power while following, 0-1. Also max speed = this x full speed. Manual drive ignore it. */
    public static double maxPower = 1.0;

    /** Brake power per in/s over speed limit. */
    public static double BRAKE_KP = 0.05;

    /** Field velocity + heading. Robot set after follower made. Null = no speed limit. */
    public static Supplier<Velocity> velocity;
    public static DoubleSupplier heading;

    public LimitedMecanum(HardwareMap map, MecanumConfig config) {
        super(map, config);
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        if (!manual && maxPower < 1.0) {
            DrivePowers brake = overSpeedBrake();
            if (brake != null) {
                // Pedro float motors on path. Low power alone only coast. Push back.
                super.drive(new DrivePowers(brake.forward(), brake.strafe(), powers.turn() * maxPower), manual);
                return;
            }
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

    /** Too fast for maxPower: power against motion, robot frame. Else null. */
    private DrivePowers overSpeedBrake() {
        if (velocity == null || heading == null) {
            return null;
        }
        Velocity v = velocity.get();
        if (v == null) {
            return null;
        }
        double speed = Math.hypot(v.vx, v.vy);
        double limit = maxPower * Constants.foresightConfig.maxAchievableForwardVelocity.get();
        if (speed <= limit || speed < 1e-6) {
            return null;
        }
        double h = heading.getAsDouble();
        // Field to robot frame. Strafe + = left.
        double forwardV = v.vx * Math.cos(h) + v.vy * Math.sin(h);
        double strafeV = -v.vx * Math.sin(h) + v.vy * Math.cos(h);
        double power = Math.min(1.0, BRAKE_KP * (speed - limit));
        return new DrivePowers(-forwardV / speed * power, -strafeV / speed * power, 0);
    }
}
