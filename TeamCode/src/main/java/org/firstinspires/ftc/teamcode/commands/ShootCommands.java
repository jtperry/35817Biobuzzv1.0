package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;

/** Shot sequences (spec 5.6). Only the intake is required; turret, hood and flywheel track on their own. */
@Configurable
public final class ShootCommands {

    /** True = stages run only while ready: pause on an RPM dip or aim loss. False = v1 straight feed once ready. */
    public static boolean GATE_PER_BALL = true;

    /** Pause feeding when RPM falls this far below target. */
    public static double DIP_RPM = 150;

    /** Never ready after this long: feed anyway, ungated (v1 behavior). Once ready, gating stays on. */
    public static double READY_TIMEOUT_MS = 2000;

    /** A volley ends after this much actual feeding time... */
    public static double VOLLEY_FEED_MS = 1000;

    /** ...or after this long in total. */
    public static double VOLLEY_TIMEOUT_MS = 3500;

    private ShootCommands() {
    }

    /** One volley, then gate closed and stages off (auto). */
    public static Command volley(Robot robot) {
        FeedGate gate = new FeedGate(robot);
        return Commands.instant(gate::reset)
                .then(robot.intake.feed(gate::feeding).until(gate::volleyDone))
                .then(robot.intake.stopFeed());
    }

    /** Feed while held. Bind with whileTrue(...) and onFalse(robot.intake.stopFeed()). */
    public static Command whileHeld(Robot robot) {
        FeedGate gate = new FeedGate(robot);
        return Commands.instant(gate::reset).then(robot.intake.feed(gate::feeding));
    }

    /** Decides each loop whether the stages may push balls, and when a volley is done. */
    static final class FeedGate {

        private final BooleanSupplier ready;
        private final BooleanSupplier aimSettled;
        private final DoubleSupplier rpmDeficit;
        private final LongSupplier clockNanos;
        private boolean feeding = false;
        private boolean forced = false;
        private boolean everReady = false;
        private long startNanos = 0;
        private long lastNanos = 0;
        private double fedMs = 0;

        FeedGate(Robot robot) {
            this(robot::readyToShoot, robot::aimSettled, robot.flywheel::rpmDeficit, System::nanoTime);
        }

        /** Readiness, aim, RPM deficit and clock as suppliers (unit tests). */
        FeedGate(BooleanSupplier ready, BooleanSupplier aimSettled, DoubleSupplier rpmDeficit, LongSupplier clockNanos) {
            this.ready = ready;
            this.aimSettled = aimSettled;
            this.rpmDeficit = rpmDeficit;
            this.clockNanos = clockNanos;
        }

        void reset() {
            feeding = false;
            forced = false;
            everReady = false;
            fedMs = 0;
            startNanos = clockNanos.getAsLong();
            lastNanos = startNanos;
        }

        boolean feeding() {
            long now = clockNanos.getAsLong();
            if (feeding) {
                fedMs += (now - lastNanos) / 1e6;
            }
            lastNanos = now;
            if (forced) {
                feeding = true;
            } else if (!feeding) {
                feeding = ready.getAsBoolean();
                everReady |= feeding;
                // Never ready in this run: feed anyway. Once ready, dips and aim loss keep pausing.
                if (!everReady && (now - startNanos) / 1e6 >= READY_TIMEOUT_MS) {
                    forced = true;
                    feeding = true;
                }
            } else if (GATE_PER_BALL && (rpmDeficit.getAsDouble() > DIP_RPM || !aimSettled.getAsBoolean())) {
                feeding = false;
            }
            return feeding;
        }

        boolean volleyDone() {
            return fedMs >= VOLLEY_FEED_MS || (clockNanos.getAsLong() - startNanos) / 1e6 >= VOLLEY_TIMEOUT_MS;
        }
    }
}
