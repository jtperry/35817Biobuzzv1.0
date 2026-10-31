package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

import com.pedropathing.ivy.commands.Commands;

import com.bylazar.telemetry.PanelsTelemetry;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/** Flywheel tuner. Finds kS + kV in volts. No balls. */
@NextTeleop(name = "Flywheel Tuner", group = "test")
public class FlywheelTuner extends NextOpMode {

    /** Powers to test, low to high. Shooting range only (~2500-4000 RPM), line most exact there. */
    private static final double[] POWERS = {0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75};

    /** Wait this long at each power for speed to settle. */
    private static final double SETTLE_S = 2.5;

    /** Then average RPM + battery this long. */
    private static final double MEASURE_S = 1.0;

    private final Robot robot;
    private final CommandGamepad driver;

    private boolean running = false;
    private int step = 0;
    private long stepStartNanos = 0;
    private double power = 0;

    private double rpmSum = 0;
    private double voltsSum = 0;
    private int samples = 0;

    /** Done points: {RPM, applied volts, power, battery}. */
    private final List<double[]> points = new ArrayList<>();

    private double kS = Double.NaN;
    private double kV = Double.NaN;
    private double r2 = Double.NaN;

    public FlywheelTuner(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;

        Telemetry.addBackend(PanelsTelemetry.INSTANCE.getFtcTelemetry());
        robot.bindHardware();
        // Follower setup also find battery sensor.
        robot.initFollower(hardwareMap, Constants.DEFAULT_START_POSE);

        driver = new CommandGamepad(gamepad1);
        driver.a().onTrue(Commands.instant(this::begin));
        driver.b().onTrue(Commands.instant(this::abort));
    }

    @Override
    public void start() {
        // Raw power only. Speed control off, turret + hood still.
        robot.shooter.manualControl(() -> power, null, null).schedule();
    }

    private void begin() {
        if (running) {
            return;
        }
        points.clear();
        kS = kV = r2 = Double.NaN;
        step = 0;
        running = true;
        startStep();
    }

    private void abort() {
        running = false;
        power = 0;
    }

    private void startStep() {
        power = POWERS[step];
        stepStartNanos = System.nanoTime();
        rpmSum = voltsSum = 0;
        samples = 0;
    }

    @Override
    public void periodic() {
        if (running) {
            runStep();
        }
        pushTelemetry();
    }

    private void runStep() {
        double t = (System.nanoTime() - stepStartNanos) / 1e9;
        if (t < SETTLE_S) {
            return;
        }
        if (t < SETTLE_S + MEASURE_S) {
            rpmSum += robot.shooter.currentRpm();
            voltsSum += robot.batteryVoltage();
            samples++;
            return;
        }

        double rpm = rpmSum / Math.max(samples, 1);
        double battery = voltsSum / Math.max(samples, 1);
        // Motor get power x battery volts.
        points.add(new double[]{rpm, power * battery, power, battery});
        // Also to logcat. Survive after OpMode stop.
        android.util.Log.i("FlywheelTuner", String.format(
                "power %.2f  rpm %.0f  appliedV %.3f  battery %.2f", power, rpm, power * battery, battery));

        step++;
        if (step < POWERS.length) {
            startStep();
        } else {
            running = false;
            power = 0;
            fitLine();
        }
    }

    /** Least squares line: volts = kS + kV * RPM. */
    private void fitLine() {
        int n = points.size();
        if (n < 2) {
            return;
        }
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (double[] p : points) {
            sx += p[0];
            sy += p[1];
            sxx += p[0] * p[0];
            sxy += p[0] * p[1];
        }
        double denom = n * sxx - sx * sx;
        if (Math.abs(denom) < 1e-9) {
            return;
        }
        kV = (n * sxy - sx * sy) / denom;
        kS = (sy - kV * sx) / n;

        // Fit quality. 1.0 = perfect line.
        double mean = sy / n;
        double ssTot = 0, ssRes = 0;
        for (double[] p : points) {
            double predicted = kS + kV * p[0];
            ssRes += (p[1] - predicted) * (p[1] - predicted);
            ssTot += (p[1] - mean) * (p[1] - mean);
        }
        r2 = ssTot > 0 ? 1 - ssRes / ssTot : Double.NaN;
    }

    private void pushTelemetry() {
        Telemetry.log("A = start, B = stop. No balls in robot.");
        Telemetry.log("drive: gamepad 2");
        if (running) {
            Telemetry.log("step", (step + 1) + " / " + POWERS.length + "   power " + power);
        } else {
            Telemetry.log("step", points.isEmpty() ? "idle" : "done");
        }
        Telemetry.log("RPM now", String.format("%.0f", robot.shooter.currentRpm()));
        Telemetry.log("battery (V)", String.format("%.2f", robot.batteryVoltage()));
        Telemetry.log("");

        for (double[] p : points) {
            Telemetry.log(String.format("power %.1f", p[2]),
                    String.format("%.0f RPM   %.2f V applied   (battery %.2f)", p[0], p[1], p[3]));
        }

        if (!Double.isNaN(kV)) {
            Telemetry.log("");
            Telemetry.log("FLYWHEEL_KS_VOLTS", String.format("%.4f", kS));
            Telemetry.log("FLYWHEEL_KV_VOLTS_PER_RPM", String.format("%.7f", kV));
            Telemetry.log("fit (1.0 = perfect)", String.format("%.4f", r2));
            Telemetry.log("copy into Constants.java");
        }

        robot.driveRobotCentric(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x);
    }

    @Override
    public void end() {
        power = 0;
        robot.shooter.coastFlywheel();
    }
}
