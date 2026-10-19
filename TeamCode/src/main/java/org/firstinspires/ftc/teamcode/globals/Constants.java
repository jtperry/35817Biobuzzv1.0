package org.firstinspires.ftc.teamcode.globals;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import dev.nextftc.control.util.InterpolatingMap;
import kotlin.jvm.functions.Function3;

/** Tuning numbers + field geometry. */
@Configurable
public final class Constants {

    // Tunables: static, not final. Panels need it.

    private Constants() {
    }

    // ---------------------------------------------------------------- field geometry

    /** Field size, inches. Pedro field 0..144 both axes. */
    public static final double FIELD_SIZE_IN = 144;

    /** Goal A spot, inches, BLUE alliance. Red = mirrored across x = 72. */
    public static double GOAL_A_X = 87.3;
    public static double GOAL_A_Y = 86.4;

    /** Goal B spot, inches, BLUE alliance. Start = A mirrored across y = 72. */
    public static double GOAL_B_X = 80.3;
    public static double GOAL_B_Y = 57.6;

    /** Teleop goal pick: robot must pass halfway line by this much to switch. Stop flicker. */
    public static double GOAL_SWITCH_DEADBAND_IN = 2.0;

    /** Robot center to left edge, inches. From CAD. */
    public static final double ROBOT_CENTER_TO_LEFT_IN = 7.22;

    /** Robot center to back edge, inches. From CAD. */
    public static final double ROBOT_CENTER_TO_BACK_IN = 7.31;

    /** Start pose when OpMode give none. Top right corner, left side on blue wall, back on top wall. */
    public static final Pose DEFAULT_START_POSE = new Pose(
            FIELD_SIZE_IN - ROBOT_CENTER_TO_LEFT_IN, FIELD_SIZE_IN - ROBOT_CENTER_TO_BACK_IN, Math.toRadians(-90));

    // ---------------------------------------------------------------- turret geometry
    //
    // turret deg per servo deg = (60 / 20) * (65 / 169) = 15 / 13 ~= 1.154

    /** One turret servo full travel, degrees. */
    public static double TURRET_SERVO_TRAVEL_DEG = 320;

    /** Turret servo pulse range, us. Match SWYFT programmer. */
    public static final double TURRET_SERVO_PWM_MIN_US = 500;
    public static final double TURRET_SERVO_PWM_MAX_US = 2500;

    /** Turret servo directions. */
    public static final boolean TURRET_LEFT_REVERSED = false;
    public static final boolean TURRET_RIGHT_REVERSED = false;

    public static final double TURRET_SERVO_GEAR_TEETH = 60;
    public static final double TURRET_SHAFT_INPUT_GEAR_TEETH = 20;
    public static final double TURRET_SHAFT_OUTPUT_GEAR_TEETH = 65;
    public static final double TURRET_RING_GEAR_TEETH = 169;

    /** Turret deg per servo deg. 15/13 now. */
    public static double turretDegPerServoDeg() {
        return (TURRET_SERVO_GEAR_TEETH / TURRET_SHAFT_INPUT_GEAR_TEETH)
                * (TURRET_SHAFT_OUTPUT_GEAR_TEETH / TURRET_RING_GEAR_TEETH);
    }

    /** Total turret travel. ~369 deg at 320 servo. */
    public static double turretTravelDeg() {
        return TURRET_SERVO_TRAVEL_DEG * turretDegPerServoDeg();
    }

    /** Servo position per turret degree. */
    public static double turretPositionPerDeg() {
        return 1.0 / turretTravelDeg();
    }

    // ---------------------------------------------------------------- turret calibration

    public static final double TURRET_CENTER_DEG = 0;

    /** Servo position, turret straight forward. */
    public static double TURRET_CENTER_POSITION = 0.5;

    /** Gap kept from each end of travel. */
    public static double TURRET_LIMIT_MARGIN_DEG = 2.0;

    /** Turret limits, deg from robot forward. */
    public static double turretMinDeg() {
        return Math.max(-180, TURRET_CENTER_DEG - TURRET_CENTER_POSITION / turretPositionPerDeg()
                + TURRET_LIMIT_MARGIN_DEG);
    }

    public static double turretMaxDeg() {
        return Math.min(180, TURRET_CENTER_DEG + (1 - TURRET_CENTER_POSITION) / turretPositionPerDeg()
                - TURRET_LIMIT_MARGIN_DEG);
    }

    // ---------------------------------------------------------------- turret timing

    /** Turret turn speed, deg/sec. Used to guess where turret is. */
    public static double TURRET_SLEW_DEG_PER_SEC = 500;

    /** Guessed turret within this of target = settled, ok to shoot. */
    public static double TURRET_SETTLE_TOLERANCE_DEG = 3.0;

    /** manualTurret() deg per loop at full input. */
    public static double TURRET_TRIM_DEG_PER_LOOP = 1.5;

    /** Turret trim deg per press. */
    public static double TURRET_TRIM_STEP_DEG = 1.0;

    /** Shooter Test turret speed at full stick, deg/sec. */
    public static double TURRET_STICK_DEG_PER_SEC = 250;

    /** Stick below this ignored. */
    public static double STICK_DEADZONE = 0.05;

    // ---------------------------------------------------------------- hood

    public static double HOOD_MIN_POSITION = 0.0254;
    public static double HOOD_MAX_POSITION = 0.45;
    public static double HOOD_STOWED_POSITION = 0.0;

    /** Hood trim per press. */
    public static double HOOD_TRIM_STEP = 0.01;

    /** Shooter Test hood speed at full stick, position/sec. */
    public static double HOOD_STICK_POSITION_PER_SEC = 0.5;

    // ---------------------------------------------------------------- flywheel

    /** Flywheel encoder ticks per rev. */
    public static double FLYWHEEL_TICKS_PER_REV = 28.0;

    /** Flywheel velocity gains. */
    public static double FLYWHEEL_KP = 0.004;
    public static double FLYWHEEL_KI = 0.0;
    public static double FLYWHEEL_KD = 0.0;
    public static double FLYWHEEL_KS = 0.0;
    public static double FLYWHEEL_KV = 1.0 / 6000.0;

    /** RPM error bigger than this = bang-bang (full power or off). Smaller = PIDF. */
    public static double FLYWHEEL_BANG_BANG_ERROR_RPM = 100;

    /** Idle speed between shots. */
    public static double IDLE_RPM = 1000;

    /** This close to target = at speed. */
    public static double RPM_TOLERANCE = 100;

    /** Stay in tolerance this long before atSpeed() true. */
    public static double AT_SPEED_DEBOUNCE_SECONDS = 0.1;

    // ---------------------------------------------------------------- gate

    public static double GATE_OPEN_POSITION = 0.42;
    public static double GATE_CLOSED_POSITION = 0.23;

    /** Wait after gate open before stages feed. Gate get out of way. */
    public static double GATE_OPEN_DELAY_MS = 100;

    // ---------------------------------------------------------------- flower arm

    public static double FLOWER_STOWED_POSITION = 0.1;
    public static double FLOWER_DEPLOYED_POSITION = 0.8;

    // ---------------------------------------------------------------- intake powers

    // Positive = toward shooter, both stages. Motor mirroring handled by signs below.

    /** Motor sign that move stage toward shooter. Stages mounted opposite. */
    public static final double STAGE1_SIGN = -1.0;
    public static final double STAGE2_SIGN = 1.0;

    /** Intaking: stage 1 pull in. */
    public static double STAGE1_INTAKE_POWER = 1.0;

    /** Intaking: stage 2 half speed backwards. */
    public static double STAGE2_INTAKE_POWER = -0.5;

    /** Feeding: stage 1 toward shooter. Lower = slower shots. */
    public static double STAGE1_FEED_POWER = 0.7;

    /** Feeding: stage 2 toward shooter. Lower = slower shots. */
    public static double STAGE2_FEED_POWER = 1.0;

    /** Eject: both stages away from shooter. */
    public static double EJECT_POWER = -1.0;

    // ---------------------------------------------------------------- timing

    /** Gate open time for one artifact. */
    public static double FEED_TIME_MS = 250;

    /** Both stages run time to empty robot. */
    public static double VOLLEY_TIME_MS = 500;

    /** Max wait for flywheel speed. */
    public static double SPINUP_TIMEOUT_MS = 2000;

    // ---------------------------------------------------------------- drivetrain
    //
    // Drive motors set in pedro/Constants.java.

    // ---------------------------------------------------------------- shoot on the move

    /** True = lead goal when robot moving. */
    public static boolean SHOOT_ON_MOVE = true;

    /** Time from aim to ball leaving shooter, seconds. Robot keep moving this long. */
    public static double SOTM_LATENCY_S = 0.1;

    /** Lead multiplier. More = aim further ahead. Raise if shots land behind motion. */
    public static double SOTM_LEAD_GAIN = 1.5;

    /** Flywheel wheel diameter, inches. 72mm Rhino wheel, from CAD. */
    public static final double FLYWHEEL_DIAMETER_IN = 72 / 25.4;

    /** Ball exit speed / flywheel surface speed. Ball roll on hood = about half. */
    public static double BALL_EXIT_EFFICIENCY = 0.5;

    /** Ball leave shooter this high off floor, inches. Flywheel center, from CAD. */
    public static double LAUNCH_HEIGHT_IN = 12.3;

    /** Goal height, inches. Middle of up-facing CELL opening: 53.5 to 65.6 in (manual fig 9-10). */
    public static double GOAL_HEIGHT_IN = 59.5;

    /** Gravity, inches per second squared. */
    public static final double GRAVITY_IN_PER_S2 = 386.09;

    // ---------------------------------------------------------------- lookup tables

    /** Blend between two table entries. */
    private static final Function3<Double, Double, Double, Double> LERP =
            (lower, upper, t) -> lower + (upper - lower) * t;

    /** Goal distance (inches) -> flywheel RPM. Placeholder. */
    public static final InterpolatingMap<Double> DISTANCE_TO_RPM = new InterpolatingMap<>(LERP);

    /** Goal distance (inches) -> hood position. Placeholder. */
    public static final InterpolatingMap<Double> DISTANCE_TO_HOOD = new InterpolatingMap<>(LERP);

    static {
        // Placeholders. Replace when tuning.
        DISTANCE_TO_RPM.put(20.7, 2900.0);
        DISTANCE_TO_RPM.put(34.9, 3000.0);
        DISTANCE_TO_RPM.put(42.5, 3000.0);
        DISTANCE_TO_RPM.put(51.3, 3000.0);
        DISTANCE_TO_RPM.put(55.1, 3200.0);
        DISTANCE_TO_RPM.put(60.7, 3200.0);
        DISTANCE_TO_RPM.put(64.7, 3250.0);
        DISTANCE_TO_RPM.put(70.2, 3300.0);
        DISTANCE_TO_RPM.put(91.1, 3500.0);

        DISTANCE_TO_HOOD.put(20.7, 0.03);
        DISTANCE_TO_HOOD.put(34.9, 0.2);
        DISTANCE_TO_HOOD.put(42.5, 0.22);
        DISTANCE_TO_HOOD.put(51.3, 0.31);
        DISTANCE_TO_HOOD.put(55.1, 0.35);
        DISTANCE_TO_HOOD.put(60.7, 0.41);
        DISTANCE_TO_HOOD.put(64.7, 0.42);
        DISTANCE_TO_HOOD.put(70.2, 0.45);
        DISTANCE_TO_HOOD.put(91.1, 0.45);

    }

    // ---------------------------------------------------------------- helpers

    /** Goal x for alliance + goal. */
    public static double goalX(Alliance alliance, Goal goal) {
        double blueX = goal == Goal.B ? GOAL_B_X : GOAL_A_X;
        return alliance == Alliance.RED ? FIELD_SIZE_IN - blueX : blueX;
    }

    /** Goal y for alliance + goal. */
    public static double goalY(Alliance alliance, Goal goal) {
        return goal == Goal.B ? GOAL_B_Y : GOAL_A_Y;
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /** Wrap degrees to [-180, 180). */
    public static double normalizeDeg(double degrees) {
        double wrapped = (degrees + 180) % 360;
        if (wrapped < 0) {
            wrapped += 360;
        }
        return wrapped - 180;
    }
}
