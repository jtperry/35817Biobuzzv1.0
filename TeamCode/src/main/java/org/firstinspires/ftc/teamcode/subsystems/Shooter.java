package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Goal;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import dev.nextftc.control.filters.Debouncer;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

/** Shooter. Flywheel, hood, turret. */
public class Shooter implements Mechanism {

    /** Flywheel motor. */
    private final NextMotor flywheel = new NextMotor(
            "flywheel", Units.Rotations.of(1.0 / Constants.FLYWHEEL_TICKS_PER_REV));
    // Not final. bindServos() replace each OpMode.
    private NextServo hood;
    private NextServo turretLeft;
    private NextServo turretRight;

    private final Supplier<Pose> poseSupplier;
    private final Supplier<Alliance> allianceSupplier;
    private final Supplier<Goal> goalSupplier;
    private final Supplier<Velocity> velocitySupplier;
    private final DoubleSupplier voltageSupplier;

    private final Debouncer atSpeedDebouncer =
            new Debouncer(Constants.AT_SPEED_DEBOUNCE_SECONDS, Debouncer.DebounceType.RISING);

    /** Flywheel speed. Only truth. */
    private double targetRpm = 0;

    /** Armed: autoAim() track RPM from distance table. */
    private boolean armed = false;

    private double commandedTurretDeg = Constants.TURRET_CENTER_DEG;
    private double commandedHoodPosition = Constants.HOOD_STOWED_POSITION;
    private boolean atSpeed = false;

    /** Driver trim. Added on top of tracking. Stay across shots. */
    private double turretOffsetDeg = 0;
    private double hoodOffset = 0;

    /** Guess of real turret angle. Chase commanded at slew speed. Servo no report. */
    private double estimatedTurretDeg = Constants.TURRET_CENTER_DEG;
    private long lastEstimateNanos = 0;

    /** True: manualTurret() own turret. Tracking off. */
    private boolean manualOverride = false;

    /** Ticks-per-rev encoder scaling last built from. */
    private double appliedTicksPerRev = Constants.FLYWHEEL_TICKS_PER_REV;

    /** True: periodic() drive flywheel to targetRpm. False: raw power from test. */
    private boolean flywheelClosedLoop = true;

    private double flywheelIntegral = 0;
    private double lastFlywheelError = 0;
    private long lastFlywheelNanos = 0;

    public Shooter(Supplier<Pose> poseSupplier, Supplier<Alliance> allianceSupplier,
                   Supplier<Goal> goalSupplier, Supplier<Velocity> velocitySupplier,
                   DoubleSupplier voltageSupplier) {
        this.poseSupplier = poseSupplier;
        this.allianceSupplier = allianceSupplier;
        this.goalSupplier = goalSupplier;
        this.velocitySupplier = velocitySupplier;
        this.voltageSupplier = voltageSupplier;
        bindServos();
        applyTuning();
    }

    /** Make fresh hood + turret servos. Call every OpMode start. */
    public void bindServos() {
        hood = new NextServo("hood");
        turretLeft = new NextServo("turretLeft");
        turretRight = new NextServo("turretRight");
        configureTurretServo(turretLeft, Constants.TURRET_LEFT_REVERSED);
        configureTurretServo(turretRight, Constants.TURRET_RIGHT_REVERSED);
    }

    /** Set turret servo pulse range + direction. Apply when hardware first used. */
    private static void configureTurretServo(NextServo servo, boolean reversed) {
        servo.setPwmRange(Constants.TURRET_SERVO_PWM_MIN_US, Constants.TURRET_SERVO_PWM_MAX_US);
        servo.setDirection(reversed ? NextMotor.Direction.REVERSE : NextMotor.Direction.FORWARD);
    }

    /** Copy tunables into encoder scaling + debouncer. */
    private void applyTuning() {
        // Rebuild only on change. No new Angle every loop.
        if (Constants.FLYWHEEL_TICKS_PER_REV != appliedTicksPerRev) {
            flywheel.setAnglePerCount(Units.Rotations.of(1.0 / Constants.FLYWHEEL_TICKS_PER_REV));
            appliedTicksPerRev = Constants.FLYWHEEL_TICKS_PER_REV;
        }

        atSpeedDebouncer.setDebounceTimeSeconds(Constants.AT_SPEED_DEBOUNCE_SECONDS);
    }

    // ---------------------------------------------------------------- lifecycle

    /** Known state. Clear leftover state from last OpMode. */
    public void reset() {
        armed = false;
        manualOverride = false;
        turretOffsetDeg = 0;
        hoodOffset = 0;
        targetRpm = 0;
        atSpeed = false;
        atSpeedDebouncer.calculate(false);
        flywheelClosedLoop = true;
        flywheelIntegral = 0;
        estimatedTurretDeg = Constants.TURRET_CENTER_DEG;
        lastEstimateNanos = 0;
        applyTurret(Constants.TURRET_CENTER_DEG);
        applyHood(Constants.HOOD_STOWED_POSITION);
        applyFlywheel();
    }

    @Override
    public void periodic() {
        applyTuning();
        if (flywheelClosedLoop) {
            flywheel.setThrottle(flywheelPower());
        }
        atSpeed = atSpeedDebouncer.calculate(withinRpmTolerance());
        updateTurretEstimate();
    }

    /** Move turret guess toward commanded, max slew speed. */
    private void updateTurretEstimate() {
        long now = System.nanoTime();
        // Cap. Slow loop no big jump.
        double dt = lastEstimateNanos == 0 ? 0 : Math.min((now - lastEstimateNanos) / 1e9, 0.1);
        lastEstimateNanos = now;
        double maxStep = Constants.TURRET_SLEW_DEG_PER_SEC * dt;
        double error = commandedTurretDeg - estimatedTurretDeg;
        estimatedTurretDeg += Constants.clamp(error, -maxStep, maxStep);
    }

    @Override
    public Command getDefaultCommand() {
        return autoAim();
    }

    // ---------------------------------------------------------------- state

    /** Distance to aim point. Moving = led goal. */
    public double distanceToGoal() {
        double[] shot = shotGeometry();
        return Math.hypot(shot[0], shot[1]);
    }

    /** Aim point bearing, degrees from robot forward. Moving = led goal. */
    public double bearingToGoal() {
        double[] shot = shotGeometry();
        double fieldBearingDeg = Math.toDegrees(Math.atan2(shot[1], shot[0]));
        return Constants.normalizeDeg(fieldBearingDeg - Math.toDegrees(shot[2]));
    }

    /** Shot geometry: {dx, dy, heading}. Shooter spot to aim point, field frame. */
    private double[] shotGeometry() {
        Pose pose = poseSupplier.get();
        Alliance alliance = allianceSupplier.get();
        Goal goal = goalSupplier.get();
        double goalX = Constants.goalX(alliance, goal);
        double goalY = Constants.goalY(alliance, goal);
        double x = pose.x();
        double y = pose.y();
        double heading = pose.heading();

        Velocity velocity = velocitySupplier.get();
        if (Constants.SHOOT_ON_MOVE && velocity != null) {
            // Robot keep moving till ball leave. Shoot from there.
            x += velocity.vx * Constants.SOTM_LATENCY_S;
            y += velocity.vy * Constants.SOTM_LATENCY_S;
            heading += velocity.omega * Constants.SOTM_LATENCY_S;

            // Ball keep robot speed in air. Aim at goal minus drift.
            // Drift need flight time, flight time need distance. Few loops settle.
            double aimX = goalX;
            double aimY = goalY;
            for (int i = 0; i < 3; i++) {
                double flightS = flightTimeS(Math.hypot(aimX - x, aimY - y));
                aimX = goalX - velocity.vx * flightS * Constants.SOTM_LEAD_GAIN;
                aimY = goalY - velocity.vy * flightS * Constants.SOTM_LEAD_GAIN;
            }
            goalX = aimX;
            goalY = aimY;
        }
        return new double[]{goalX - x, goalY - y, heading};
    }

    public boolean inTurretRange(double degrees) {
        return degrees >= Constants.turretMinDeg() && degrees <= Constants.turretMaxDeg();
    }

    public double currentRpm() {
        return flywheel.getEncoderVelocity().into(Units.RotationsPerMinute);
    }

    public double targetRpm() {
        return targetRpm;
    }

    public boolean atSpeed() {
        return atSpeed;
    }

    public boolean isArmed() {
        return armed;
    }

    /** Motor power now. */
    public double flywheelThrottle() {
        return flywheel.getThrottle();
    }

    /** Cut flywheel now, no command. For OpMode end. */
    public void coastFlywheel() {
        targetRpm = 0;
        armed = false;
        flywheelClosedLoop = true;
        flywheel.setThrottle(0);
    }

    /** Flywheel amps. */
    public double flywheelCurrentAmps() {
        return flywheel.getCurrent().into(Units.Amperes);
    }

    /** Guessed turret close to target. Tracking moving goal still count. */
    public boolean turretSettled() {
        return Math.abs(commandedTurretDeg - estimatedTurretDeg) <= Constants.TURRET_SETTLE_TOLERANCE_DEG;
    }

    public boolean readyToShoot() {
        return targetRpm > 0 && atSpeed() && turretSettled();
    }

    public double commandedTurretDeg() {
        return commandedTurretDeg;
    }

    public double commandedHoodPosition() {
        return commandedHoodPosition;
    }

    // ---------------------------------------------------------------- commands

    /** Default command. Aim at goal, refresh flywheel. Other commands interrupt, it come back. */
    public Command autoAim() {
        return infinite(this::aimAtGoal);
    }

    /** Aim turret + hood at goal. Done when turret arrive. */
    public Command aim() {
        // Aim in start too, so settle timer running before first done check.
        return infinite(this::aimAtGoal)
                .setStart(this::aimAtGoal)
                .setDone(this::turretSettled);
    }

    /** One aim step. Turret + hood from pose, flywheel refreshed. */
    private void aimAtGoal() {
        double distance = distanceToGoal();

        // Manual override take turret only. Hood keep tracking.
        if (!manualOverride) {
            applyTurret(Constants.clamp(bearingToGoal() + turretOffsetDeg,
                    Constants.turretMinDeg(), Constants.turretMaxDeg()));
        }
        applyHood(Constants.clamp(
                lookUp(Constants.DISTANCE_TO_HOOD, distance, commandedHoodPosition) + hoodOffset,
                Constants.HOOD_MIN_POSITION, Constants.HOOD_MAX_POSITION));
        if (armed) {
            targetRpm = lookUp(Constants.DISTANCE_TO_RPM, distance, targetRpm);
        }
        applyFlywheel();
    }

    /** Init aim. Turret point at goal, no command, flywheel untouched. Call each init loop. */
    public void aimTurretAtGoal() {
        applyTurret(Constants.clamp(bearingToGoal() + turretOffsetDeg,
                Constants.turretMinDeg(), Constants.turretMaxDeg()));
    }

    /** Shot tuning. Turret aim at goal. RPM + hood from suppliers, not tables. */
    public Command tuneShot(DoubleSupplier rpm, DoubleSupplier hoodPosition) {
        return infinite(() -> {
            applyTurret(Constants.clamp(bearingToGoal(),
                    Constants.turretMinDeg(), Constants.turretMaxDeg()));
            applyHood(Constants.clamp(hoodPosition.getAsDouble(),
                    Constants.HOOD_MIN_POSITION, Constants.HOOD_MAX_POSITION));
            armed = false;
            targetRpm = Math.max(0, rpm.getAsDouble());
            applyFlywheel();
        });
    }

    /** Arm, no command. Flywheel hold table RPM from next aim loop. */
    public void arm() {
        armed = true;
    }

    /** Arm. Spin to table RPM for current distance. */
    public Command spinUp() {
        return instant(() -> {
            armed = true;
            targetRpm = lookUp(Constants.DISTANCE_TO_RPM, distanceToGoal(), targetRpm);
            applyFlywheel();
        });
    }

    /** Fixed RPM, ignore pose. */
    public Command spinUpTo(double rpm) {
        return instant(() -> {
            armed = false;
            targetRpm = rpm;
            applyFlywheel();
        });
    }

    /** Drop to idle RPM, not zero. Next spin-up faster. */
    public Command idleFlywheel() {
        return instant(() -> {
            armed = false;
            targetRpm = Constants.IDLE_RPM;
            applyFlywheel();
        });
    }

    /** Do nothing, own shooter. autoAim() off, nothing move. */
    public Command standby() {
        return infinite(() -> {
        });
    }

    public Command stopFlywheel() {
        return instant(() -> {
            armed = false;
            targetRpm = 0;
            applyFlywheel();
        });
    }

    /** Raw control for testing. Flywheel power, turret + hood servo position. Null = leave alone. */
    public Command manualControl(
            DoubleSupplier power, DoubleSupplier turretPosition, DoubleSupplier hoodPosition) {
        return infinite(() -> {
            if (power != null) {
                // Raw power. Speed control off.
                flywheelClosedLoop = false;
                flywheel.setThrottle(Constants.clamp(power.getAsDouble(), -1.0, 1.0));
            }

            if (turretPosition != null) {
                double turret = Constants.clamp(turretPosition.getAsDouble(), 0, 1);
                // Directions set in bindServos(). Both get same value.
                turretLeft.setPosition(turret);
                turretRight.setPosition(turret);
            }

            if (hoodPosition != null) {
                hood.setPosition(Constants.clamp(hoodPosition.getAsDouble(), 0, 1));
            }
        });
    }

    /** Turret angle current constants think a raw position is. */
    public static double degreesForTurretPosition(double position) {
        return Constants.TURRET_CENTER_DEG
                + (position - Constants.TURRET_CENTER_POSITION) / Constants.turretPositionPerDeg();
    }

    /** Driver nudge turret. Interrupt autoAim(), it come back on release. */
    public Command manualTurret(DoubleSupplier input) {
        return infinite(() -> {
            manualOverride = true;
            double delta = input.getAsDouble() * Constants.TURRET_TRIM_DEG_PER_LOOP;
            applyTurret(Constants.clamp(commandedTurretDeg + delta,
                    Constants.turretMinDeg(), Constants.turretMaxDeg()));
            applyFlywheel();
        }).setEnd(endCondition -> manualOverride = false);
    }

    public Command turretTo(double degrees) {
        return instant(() -> applyTurret(Constants.clamp(degrees,
                Constants.turretMinDeg(), Constants.turretMaxDeg())));
    }

    public Command centerTurret() {
        return turretTo(Constants.TURRET_CENTER_DEG);
    }

    /** Set hood position. */
    public Command hoodTo(double position) {
        return instant(() -> applyHood(Constants.clamp(position,
                Constants.HOOD_MIN_POSITION, Constants.HOOD_MAX_POSITION)));
    }

    /** Shift hood trim. Tracking keep going. */
    public Command trimHood(double delta) {
        return instant(() -> hoodOffset += delta);
    }

    /** Shift turret trim. */
    public Command trimTurret(double deltaDeg) {
        return instant(() -> turretOffsetDeg += deltaDeg);
    }

    /** Clear override + trims. Back to pure tracking. */
    public Command resumeAutoAim() {
        return instant(() -> {
            manualOverride = false;
            turretOffsetDeg = 0;
            hoodOffset = 0;
        });
    }

    // ---------------------------------------------------------------- hardware

    private void applyTurret(double degrees) {
        commandedTurretDeg = degrees;

        double position = Constants.TURRET_CENTER_POSITION
                + (degrees - Constants.TURRET_CENTER_DEG) * Constants.turretPositionPerDeg();
        position = Constants.clamp(position, 0, 1);

        // Directions set in bindServos(). Both get same position.
        turretLeft.setPosition(position);
        turretRight.setPosition(position);
    }

    /** Speed control on. periodic() drive flywheel to targetRpm. */
    private void applyFlywheel() {
        flywheelClosedLoop = true;
    }

    /** Flywheel power for targetRpm. Far off = bang-bang. Close = PIDF. */
    private double flywheelPower() {
        long now = System.nanoTime();
        // Cap. Slow loop no big jump.
        double dt = lastFlywheelNanos == 0 ? 0 : Math.min((now - lastFlywheelNanos) / 1e9, 0.1);
        lastFlywheelNanos = now;

        if (targetRpm <= 0) {
            flywheelIntegral = 0;
            lastFlywheelError = 0;
            return 0;
        }

        double error = targetRpm - currentRpm();

        // Bang-bang: too slow = full power, too fast = off.
        if (Math.abs(error) > Constants.FLYWHEEL_BANG_BANG_ERROR_RPM) {
            flywheelIntegral = 0;
            lastFlywheelError = error;
            return error > 0 ? 1.0 : 0.0;
        }

        // PIDF: hold speed.
        flywheelIntegral += error * dt;
        double derivative = dt > 0 ? (error - lastFlywheelError) / dt : 0;
        lastFlywheelError = error;

        double power = Constants.FLYWHEEL_KS
                + Constants.FLYWHEEL_KV * targetRpm
                + Constants.FLYWHEEL_KP * error
                + Constants.FLYWHEEL_KI * flywheelIntegral
                + Constants.FLYWHEEL_KD * derivative;
        // Voltage comp. Low battery = more power for same speed.
        double volts = Math.max(voltageSupplier.getAsDouble(), 6.0);
        power *= Constants.NOMINAL_VOLTAGE / volts;
        return Constants.clamp(power, 0, 1);
    }

    private void applyHood(double position) {
        commandedHoodPosition = position;
        hood.setPosition(position);
    }

    private boolean withinRpmTolerance() {
        if (targetRpm <= 0) {
            return false;
        }
        return Math.abs(currentRpm() - targetRpm) <= Constants.RPM_TOLERANCE;
    }

    /**
     * Ball flight time to goal, seconds. Physics from table RPM.
     * Exit speed = flywheel surface speed x efficiency.
     * Launch angle = low arc that hit goal height at this distance.
     */
    public static double flightTimeS(double distanceIn) {
        double rpm = lookUp(Constants.DISTANCE_TO_RPM, distanceIn, 0);
        double speed = Constants.BALL_EXIT_EFFICIENCY
                * Math.PI * Constants.FLYWHEEL_DIAMETER_IN * rpm / 60.0;
        if (speed <= 0 || distanceIn <= 0) {
            return 0;
        }
        double g = Constants.GRAVITY_IN_PER_S2;
        double rise = Constants.GOAL_HEIGHT_IN - Constants.LAUNCH_HEIGHT_IN;
        double v2 = speed * speed;
        double root = v2 * v2 - g * (g * distanceIn * distanceIn + 2 * rise * v2);
        // Too slow to reach by physics. Estimate wrong, guess 45 deg.
        double angle = root < 0
                ? Math.PI / 4
                : Math.atan((v2 - Math.sqrt(root)) / (g * distanceIn));
        return distanceIn / (speed * Math.cos(angle));
    }

    /** Read table. Key clamped to table range. Empty table = fallback. */
    private static double lookUp(
            dev.nextftc.control.util.InterpolatingMap<Double> map, double key, double fallback) {
        if (map.isEmpty()) {
            return fallback;
        }
        // Table throw on key outside range. Clamp to nearest entry.
        double clamped = Constants.clamp(key, map.firstKey(), map.lastKey());
        Double value = map.get(clamped);
        return value == null ? fallback : value;
    }
}
