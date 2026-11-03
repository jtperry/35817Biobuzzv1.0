package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.tuning.FlywheelFit;
import org.firstinspires.ftc.teamcode.util.RobotTelemetry;

import java.util.Locale;

import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/**
 * Fits the flywheel feedforward volts = KS + KV * RPM (spec 5.1), as v1 did. Never drives. No balls in the robot.
 * Gamepad 1: A = run the power steps, B = abort (power 0).
 * Each step runs one power on motor 1, and on motor 2 when the flywheel uses it, waits SETTLE_S, then averages RPM
 * and battery volts for MEASURE_S (applied volts = power * battery). Then copy the two printed lines into Flywheel.java.
 */
@NextTeleop(name = "Flywheel Tuner", group = "test")
public class FlywheelTuner extends NextOpMode {

    /** v1's steps: about 1600-3600 RPM at 12 V with the current KS / KV. */
    private static final double[] POWERS = {0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75};
    private static final double SETTLE_S = 2.5;
    private static final double MEASURE_S = 1.0;

    private final Robot robot;
    private final RobotTelemetry tele;
    private final CommandGamepad pad;

    private final double[] stepRpm = new double[POWERS.length];
    private final double[] stepVolts = new double[POWERS.length];
    /** Steps finished; while running, also the index of the current step. */
    private int done = 0;
    private boolean running = false;
    private boolean aborted = false;
    /** Motor 2 on or off for the whole run, read once when A starts it. */
    private boolean bothMotors = false;
    private long stepStartNanos = 0;
    private double rpmSum = 0;
    private double batterySum = 0;
    private int samples = 0;
    private FlywheelFit.Result fit = null;

    public FlywheelTuner(Robot robot) {
        super(robot, BulkReadHook.INSTANCE, robot.initCheck());
        this.robot = robot;
        tele = new RobotTelemetry(telemetry);
        robot.bindHardware(hardwareMap);
        // The follower owns the OctoQuad the flywheel RPM is read from.
        robot.initFollower(hardwareMap, Field.startPose(robot.alliance()));

        pad = new CommandGamepad(gamepad1);
        pad.a().onTrue(Commands.instant(this::begin));
        pad.b().onTrue(Commands.instant(this::abort));
    }

    @Override
    public void start() {
        robot.reset();
        // Keep the turret and hood still: Robot would otherwise aim them at the cell every loop.
        robot.turret.hold(0);
        robot.hood.stow();
    }

    private void begin() {
        if (running) {
            return;
        }
        bothMotors = Flywheel.usesMotor2();
        done = 0;
        fit = null;
        aborted = false;
        running = true;
        startStep();
    }

    private void abort() {
        if (running) {
            running = false;
            aborted = true;
        }
    }

    private void startStep() {
        stepStartNanos = System.nanoTime();
        rpmSum = 0;
        batterySum = 0;
        samples = 0;
    }

    @Override
    public void periodic() {
        if (running) {
            runStep();
        }
        double power = running ? POWERS[done] : 0;
        robot.flywheel.raw(power, bothMotors ? power : 0);

        if (!tele.beginFrame()) {
            return;
        }
        tele.line("A = run the steps, B = abort. No balls in the robot.");
        // During and after a run: the motors that run used. Before one: what A would use.
        boolean both = (running || done > 0) ? bothMotors : Flywheel.usesMotor2();
        tele.add("motors", both ? "both" : "motor 1 only (set USE_MOTOR_2 first to tune both motors)");
        tele.add("state", state());
        tele.add("now", String.format("%.0f RPM   battery %.2f V", robot.flywheel.rpm(), robot.battery.volts()));
        tele.add("OctoQuad bad reads", robot.flywheel.badReads());
        for (int i = 0; i < done; i++) {
            tele.add("step " + (i + 1), String.format("power %.2f   %.3f V   %.0f RPM",
                    POWERS[i], stepVolts[i], stepRpm[i]));
        }
        if (fit != null) {
            tele.add("fit", String.format("kS %.4f V   kV %.7f V/RPM   r squared %.4f (1 = perfect line)",
                    fit.ksVolts, fit.kvVoltsPerRpm, fit.rSquared));
            if (Double.isNaN(fit.kvVoltsPerRpm)) {
                tele.line("No fit: the RPM did not change between steps. Check the OctoQuad and the encoder cable.");
            } else if (fit.kvVoltsPerRpm <= 0 || fit.ksVolts < 0) {
                tele.line("kS or kV is not positive: check ENCODER_REVERSED and the motor directions (RPM must read positive).");
            } else {
                // Locale.US: these lines are pasted into Java, which needs a dot as the decimal point.
                tele.add("paste", String.format(Locale.US, "public static double KS_VOLTS = %.4f;", fit.ksVolts));
                tele.add("paste", String.format(Locale.US, "public static double KV_VOLTS_PER_RPM = %.7f;",
                        fit.kvVoltsPerRpm));
                tele.line("Copy both lines into Flywheel.java.");
            }
        }
        tele.endFrame();
    }

    /** Settle, then average; after the last step, fit the line. */
    private void runStep() {
        double t = (System.nanoTime() - stepStartNanos) / 1e9;
        if (t < SETTLE_S) {
            return;
        }
        if (t < SETTLE_S + MEASURE_S) {
            rpmSum += robot.flywheel.rpm();
            batterySum += robot.battery.volts();
            samples++;
            return;
        }
        int n = Math.max(samples, 1);
        stepRpm[done] = rpmSum / n;
        stepVolts[done] = POWERS[done] * batterySum / n;
        done++;
        if (done < POWERS.length) {
            startStep();
        } else {
            running = false;
            fit = FlywheelFit.fit(stepRpm, stepVolts, done);
        }
    }

    private String state() {
        if (running) {
            double t = (System.nanoTime() - stepStartNanos) / 1e9;
            return String.format("step %d / %d   power %.2f   %s", done + 1, POWERS.length, POWERS[done],
                    t < SETTLE_S ? "settling" : "measuring");
        }
        if (fit != null) {
            return "done";
        }
        return aborted ? "aborted (A = run again)" : "idle (A = run)";
    }

    @Override
    public void end() {
        robot.flywheel.reset();
    }
}
