package org.firstinspires.ftc.teamcode.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;

import org.firstinspires.ftc.teamcode.pedro.SharedOctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.util.Battery;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.function.DoubleSupplier;

import dev.nextftc.control.filters.Debouncer;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

/**
 * Two motors on one flywheel; speed from one encoder on the OctoQuad, read in the same I2C transfer as the pose.
 * Speed control runs in periodic(). Mode changes are plain calls (wrap in Commands.instant at the call site).
 */
@Configurable
public class Flywheel implements Mechanism {

    public static final String MOTOR_1 = "flywheel";
    public static final String MOTOR_2 = "flywheel2";
    /** Motor directions. Flip live during bring-up (Hardware Test, Checkpoint C), then copy into code. */
    public static boolean MOTOR_1_REVERSED = false;
    public static boolean MOTOR_2_REVERSED = true;
    /** Off until Hardware Test shows both motors spin the flywheel the same way. */
    public static boolean USE_MOTOR_2 = false;

    public static final double COUNTS_PER_REV = 28.0;
    public static final int OCTOQUAD_PORT = 2;
    /** Flip if RPM reads negative while spinning forward. */
    public static boolean ENCODER_REVERSED = true;
    /** OctoQuad speed window, ms (1-255). One count per 50 ms is about 43 RPM. */
    public static int VELOCITY_WINDOW_MS = 50;

    public static double KS_VOLTS = 1.4145;
    public static double KV_VOLTS_PER_RPM = 0.0020984;
    public static double KP_VOLTS_PER_RPM = 0.01;
    public static double KI_VOLTS = 0.0;
    public static double KD_VOLTS = 0.0;
    public static double I_MAX_VOLTS = 1.5;
    /** Further below target than this = full power. */
    public static double BANG_BANG_RPM = 100;
    public static double RPM_TOLERANCE = 60;
    public static double AT_SPEED_DEBOUNCE_S = 0.1;
    public static double IDLE_RPM = 1000;

    private static final double POWER_CACHE_TOLERANCE = 0.002;

    public enum Mode { OFF, IDLE, ARMED, FIXED, RAW }

    private final NextMotor motor1 =
            new NextMotor(MOTOR_1, Units.Rotations.of(1.0 / COUNTS_PER_REV), POWER_CACHE_TOLERANCE);
    private final NextMotor motor2 =
            new NextMotor(MOTOR_2, Units.Rotations.of(1.0 / COUNTS_PER_REV), POWER_CACHE_TOLERANCE);
    private final Battery battery;
    private final Debouncer atSpeedDebouncer =
            new Debouncer(AT_SPEED_DEBOUNCE_S, Debouncer.DebounceType.RISING);

    private SharedOctoQuadLocalizer octoQuad;
    private int appliedWindowMs = -1;
    private boolean appliedReversed = false;

    private Mode mode = Mode.OFF;
    private double trackingRpm = 0;
    private DoubleSupplier fixedRpm = () -> 0;
    private double raw1 = 0;
    private double raw2 = 0;

    private double targetRpm = 0;
    private double rpm = 0;
    private boolean atSpeed = false;
    private double integral = 0;
    private double lastError = 0;
    private long lastNanos = 0;
    private int badReads = 0;

    public Flywheel(Battery battery) {
        this.battery = battery;
    }

    /** Call each OpMode after the follower exists. */
    public void useOctoQuad(SharedOctoQuadLocalizer localizer) {
        octoQuad = localizer;
        appliedWindowMs = -1;
        rpm = 0;
        applyEncoderSettings();
    }

    /** Known state for a new OpMode: off, controller cleared. */
    public void reset() {
        mode = Mode.OFF;
        trackingRpm = 0;
        raw1 = 0;
        raw2 = 0;
        targetRpm = 0;
        integral = 0;
        lastError = 0;
        lastNanos = 0;
        badReads = 0;
        atSpeed = false;
        atSpeedDebouncer.calculate(false);
        writePowers(0, 0);
    }

    @Override
    public void periodic() {
        applyEncoderSettings();
        readRpm();
        long now = System.nanoTime();
        double dt = lastNanos == 0 ? 0 : Math.min((now - lastNanos) / 1e9, 0.1);
        lastNanos = now;

        if (mode == Mode.RAW) {
            targetRpm = 0;
            writePowers(raw1, raw2);
        } else {
            targetRpm = targetFor(mode);
            double power = controlPower(dt);
            writePowers(power, USE_MOTOR_2 ? power : 0);
        }
        atSpeedDebouncer.setDebounceTimeSeconds(AT_SPEED_DEBOUNCE_S);
        atSpeed = atSpeedDebouncer.calculate(targetRpm > 0 && Math.abs(rpm - targetRpm) <= RPM_TOLERANCE);
    }

    // ---------------------------------------------------------------- modes

    public void off() {
        mode = Mode.OFF;
    }

    public void idle() {
        mode = Mode.IDLE;
    }

    /** Track the RPM Robot sets from the shot solution every loop. */
    public void arm() {
        mode = Mode.ARMED;
    }

    public void toggleArmed() {
        mode = mode == Mode.ARMED ? Mode.IDLE : Mode.ARMED;
    }

    /** Hold a supplied RPM (tuning). */
    public void fixed(DoubleSupplier rpmSupplier) {
        fixedRpm = rpmSupplier;
        mode = Mode.FIXED;
    }

    /** Raw powers per motor, no speed control (tests). Ignores USE_MOTOR_2 so each motor can be checked alone. */
    public void raw(double power1, double power2) {
        raw1 = power1;
        raw2 = power2;
        mode = Mode.RAW;
    }

    /** Robot sets this every loop from the shot solution; used while ARMED. */
    public void setTrackingRpm(double rpmValue) {
        trackingRpm = rpmValue;
    }

    // ---------------------------------------------------------------- state

    public Mode mode() {
        return mode;
    }

    public boolean isArmed() {
        return mode == Mode.ARMED;
    }

    public double rpm() {
        return rpm;
    }

    public double targetRpm() {
        return targetRpm;
    }

    public boolean atSpeed() {
        return atSpeed;
    }

    /** How far below target, RPM (negative = above target). */
    public double rpmDeficit() {
        return targetRpm - rpm;
    }

    public double power() {
        return motor1.getThrottle();
    }

    public int badReads() {
        return badReads;
    }

    /** Separate hub read each call: tests only, never in the match loop. */
    public double motor1Amps() {
        return motor1.getCurrent().into(Units.Amperes);
    }

    public double motor2Amps() {
        return motor2.getCurrent().into(Units.Amperes);
    }

    // ---------------------------------------------------------------- internals

    private double targetFor(Mode current) {
        switch (current) {
            case IDLE:
                return IDLE_RPM;
            case ARMED:
                return trackingRpm;
            case FIXED:
                return Math.max(0, fixedRpm.getAsDouble());
            default:
                return 0;
        }
    }

    /** Full power far below target; else volts feedforward + PID divided by battery volts. */
    private double controlPower(double dt) {
        if (targetRpm <= 0) {
            integral = 0;
            lastError = 0;
            return 0;
        }
        double error = targetRpm - rpm;
        if (error > BANG_BANG_RPM) {
            integral = 0;
            lastError = error;
            return 1.0;
        }
        integral += error * dt;
        if (KI_VOLTS > 0) {
            double maxIntegral = I_MAX_VOLTS / KI_VOLTS;
            integral = MathUtil.clamp(integral, -maxIntegral, maxIntegral);
        }
        double derivative = dt > 0 ? (error - lastError) / dt : 0;
        lastError = error;
        double volts = KS_VOLTS + KV_VOLTS_PER_RPM * targetRpm + KP_VOLTS_PER_RPM * error
                + KI_VOLTS * integral + KD_VOLTS * derivative;
        return MathUtil.clamp(volts / Math.max(battery.volts(), 6.0), 0, 1);
    }

    private void writePowers(double power1, double power2) {
        motor1.setThrottle(MathUtil.safePower(power1) * (MOTOR_1_REVERSED ? -1 : 1));
        motor2.setThrottle(MathUtil.safePower(power2) * (MOTOR_2_REVERSED ? -1 : 1));
    }

    private void applyEncoderSettings() {
        if (octoQuad == null) {
            return;
        }
        int window = (int) MathUtil.clamp(VELOCITY_WINDOW_MS,
                OctoQuad.MIN_VELOCITY_MEASUREMENT_INTERVAL_MS, OctoQuad.MAX_VELOCITY_MEASUREMENT_INTERVAL_MS);
        if (window != appliedWindowMs || ENCODER_REVERSED != appliedReversed) {
            // I2C writes: only when a setting changes.
            octoQuad.octoQuad.setSingleEncoderDirection(OCTOQUAD_PORT,
                    ENCODER_REVERSED ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);
            octoQuad.octoQuad.setSingleVelocitySampleInterval(OCTOQUAD_PORT, window);
            appliedWindowMs = window;
            appliedReversed = ENCODER_REVERSED;
        }
    }

    private void readRpm() {
        if (octoQuad == null || appliedWindowMs <= 0) {
            return;
        }
        OctoQuad.EncoderDataBlock data = octoQuad.encoderData();
        if (!data.crcOk) {
            badReads++;
            return;
        }
        double countsPerSecond = data.velocities[OCTOQUAD_PORT] * (1000.0 / appliedWindowMs);
        rpm = countsPerSecond / COUNTS_PER_REV * 60.0;
    }
}
