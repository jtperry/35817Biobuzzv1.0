package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
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
 * Gamepad 1: A = run the power steps, B = abort (power 0). An A held through PLAY does nothing until it is released.
 * Each step runs one power on motor 1, and on motor 2 when the flywheel uses it, waits SETTLE_S, then averages RPM
 * and battery volts for MEASURE_S (applied volts = power * battery). A step that got no samples (a loop stall longer
 * than MEASURE_S) is run again. Then copy the two printed lines into Flywheel.java.
 */
@Configurable
@NextTeleop(name = "Flywheel Tuner", group = "test")
public class FlywheelTuner extends NextOpMode {

    /** Seconds each step waits for the RPM to settle; read once per step. Not finite or not above 0: 2.5. */
    public static double SETTLE_S = 2.5;
    /** Seconds each step averages RPM and volts over; read once per step. Not finite or not above 0: 1.0. */
    public static double MEASURE_S = 1.0;

    /** v1's steps: about 1600-3600 RPM at 12 V with the current KS / KV. */
    private static final double[] POWERS = {0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75};
    private static final double DEFAULT_SETTLE_S = 2.5;
    private static final double DEFAULT_MEASURE_S = 1.0;

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
    /** SETTLE_S and MEASURE_S as validated when the current step started. */
    private double stepSettleS = DEFAULT_SETTLE_S;
    private double stepMeasureS = DEFAULT_MEASURE_S;
    /** The current step is a redo: its last measure window got no samples. */
    private boolean redoingStep = false;
    /** An A held through PLAY must be released before A starts the steps (its binding sees a press at PLAY). */
    private boolean waitForARelease = true;
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
    public void disabledPeriodic() {
        if (!tele.beginFrame()) {
            return;
        }
        tele.line("Flywheel Tuner SPINS the flywheel after PLAY + A ("
                + (Flywheel.usesMotor2() ? "motor 1 and motor 2" : "motor 1 only") + ").");
        tele.line("Take the balls out of the robot first. B = abort (power 0) at any time.");
        tele.add("each step", timing(settleS(), measureS()));
        tele.endFrame();
    }

    @Override
    public void start() {
        waitForARelease = gamepad1.a;
        robot.reset();
        // Keep the turret and hood still: Robot would otherwise aim them at the cell every loop.
        robot.turret.hold(0);
        robot.hood.stow();
    }

    private void begin() {
        if (running || waitForARelease) {
            return;
        }
        bothMotors = Flywheel.usesMotor2();
        done = 0;
        fit = null;
        aborted = false;
        redoingStep = false;
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
        stepSettleS = settleS();
        stepMeasureS = measureS();
        stepStartNanos = System.nanoTime();
        rpmSum = 0;
        batterySum = 0;
        samples = 0;
    }

    /** SETTLE_S, or its default when it is not a finite positive number. */
    private static double settleS() {
        return positiveOr(SETTLE_S, DEFAULT_SETTLE_S);
    }

    /** MEASURE_S, or its default when it is not a finite positive number. */
    private static double measureS() {
        return positiveOr(MEASURE_S, DEFAULT_MEASURE_S);
    }

    private static double positiveOr(double value, double fallback) {
        return Double.isFinite(value) && value > 0 ? value : fallback;
    }

    private static String timing(double settleS, double measureS) {
        return String.format(Locale.US, "settle %.1f s, then measure %.1f s", settleS, measureS);
    }

    @Override
    public void periodic() {
        if (!gamepad1.a) {
            waitForARelease = false;
        }
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
        // While running: this step's latched times. Otherwise: what the next step would use.
        tele.add("each step", running ? timing(stepSettleS, stepMeasureS) : timing(settleS(), measureS()));
        if (waitForARelease) {
            tele.line("Release A, then press it to run (A was held through PLAY).");
        }
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
        if (t < stepSettleS) {
            return;
        }
        if (t < stepSettleS + stepMeasureS) {
            rpmSum += robot.flywheel.rpm();
            batterySum += robot.battery.volts();
            samples++;
            return;
        }
        if (samples == 0) {
            // A loop stall longer than the measure window: a 0 RPM / 0 V point would wreck the fit. Run it again.
            redoingStep = true;
            startStep();
            return;
        }
        stepRpm[done] = rpmSum / samples;
        stepVolts[done] = POWERS[done] * batterySum / samples;
        redoingStep = false;
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
            return String.format("step %d / %d   power %.2f   %s%s", done + 1, POWERS.length, POWERS[done],
                    t < stepSettleS ? "settling" : "measuring",
                    redoingStep ? "   (redo: no samples in the last measure window)" : "");
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
