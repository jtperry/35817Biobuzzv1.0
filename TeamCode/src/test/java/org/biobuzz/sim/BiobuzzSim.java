package org.biobuzz.sim;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.intake.FlowerArm;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.shooter.Turret;
import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.config.SimRobotConfig;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeVoltageSensor;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.simloop.plant.MecanumDrivePlant;
import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.ScenarioRunner;
import org.horizon36596.simloop.simclock.SimClockLoader;

import java.lang.reflect.Field;

/**
 * Our robot on the laptop: a FakeHardwareMap holding every device our code looks up, the drive plant, the flywheel
 * model and the OctoQuad that reports them, and a runner for an OpMode adapter. Lives outside the team packages,
 * so SimLoop never reloads it: the test and every run share it (Trap 1).
 *
 * <p>Frames: the plant integrates in SimLoop's centre-origin frame (+x forward at heading 0, +y left, heading CCW);
 * Pedro's frame is the same axes moved 72 in along both, so a Pedro pose is the plant's plus 72 in (Trap 3).
 *
 * <p>Every physics number is a PLACEHOLDER until robot logs exist; each one says where it came from.
 */
public final class BiobuzzSim {

    /** The packages a run reloads onto the sim clock: ours, Pedro (Ivy included) and NextFTC (Trap 1). */
    public static final String[] TEAM_PACKAGES = {
            "org.firstinspires.ftc.teamcode.", SimClockLoader.PEDRO_PATHING, "dev.nextftc."};
    public static final int INIT_LOOP_TICKS = 25;
    /** 30 s at SimLoop's 50 Hz tick. */
    public static final int ACTIVE_TICKS = 1500;

    /** Pedro coordinates are the plant's plus this, on both axes (the field is 144 in, the plant's origin its centre). */
    public static final double PEDRO_OFFSET_IN = 72;
    public static final double FIELD_HALF_IN = 72;

    /** Pedro's drive slots [FL, FR, BL, BR] (pedro/Constants.drivetrainConfig): "rf" is the front-LEFT wheel (Trap 2). */
    public static final String[] DRIVE_MOTORS = {"rf", "lf", "rr", "lr"};
    /** How each is bolted in, [FL, FR, BL, BR]: cancels the code's REVERSE on rf and rr, so forward drives forward. */
    private static final double[] DRIVE_MOUNTING_SIGNS = {-1, 1, -1, 1};
    public static final String OCTOQUAD = "octoquad";
    public static final String VOLTAGE_SENSOR = "Control Hub";

    /**
     * The robot as a logged run names it: OpModeRunner.runLogged needs a SimRobotConfig, records its simple class
     * name in the RLOG's metadata ("SimRobotConfig") and reads nothing else from it.
     */
    public static final SimRobotConfig ROBOT_CONFIG = new BiobuzzSimConfig();

    /** PLACEHOLDER: v1 Pedro maxAchievableForwardVelocity (pedro/Constants.java), in/s at full power. */
    static final double MAX_FORWARD_IN_PER_S = 83.99482610982179;
    /** PLACEHOLDER: v1 Pedro maxAchievableStrafeVelocity, in/s. */
    static final double MAX_STRAFE_IN_PER_S = 67.03791712453315;
    /** PLACEHOLDER: v1 Pedro naturalForwardDeceleration, in/s^2 (coasting with the motors floating). */
    static final double FORWARD_COAST_DECEL = 35.11887294284997;
    /** PLACEHOLDER: v1 Pedro naturalStrafeDeceleration, in/s^2. */
    static final double STRAFE_COAST_DECEL = 62.09565526949194;
    /** PLACEHOLDER: traction limit, in/s^2 = 1 / (2 x v1 Pedro forward quadraticBrakeCoefficient). */
    static final double FORWARD_TRACTION = 1 / (2 * 0.001269610873932346);
    /** PLACEHOLDER: traction limit, in/s^2 = 1 / (2 x v1 Pedro strafe quadraticBrakeCoefficient). */
    static final double STRAFE_TRACTION = 1 / (2 * 0.0014883442411688695);
    /** PLACEHOLDER (invented): wheel speed chases power at this rate, 1/s (a 0.2 s time constant). */
    static final double DRIVE_MAX_ACCEL = 5.0;
    /** PLACEHOLDER (invented): left-right wheel centre distance, in. */
    static final double TRACK_WIDTH_IN = 13.0;
    /** PLACEHOLDER (invented): front-back wheel centre distance, in. */
    static final double WHEEL_BASE_IN = 12.0;
    /** PLACEHOLDER: a 104 mm mecanum wheel. Only sets the tick scale: our code never reads a drive encoder. */
    static final double WHEEL_DIAMETER_IN = 104 / 25.4;
    /** PLACEHOLDER: a 435 rpm goBILDA motor's encoder. Only sets the tick scale. */
    static final double DRIVE_TICKS_PER_REV = 384.5;
    static final double TICKS_PER_INCH = DRIVE_TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    /** PLACEHOLDER: positive hub power on "flywheel" spins the wheel forward (Flywheel.MOTOR_1_REVERSED = false). */
    static final double FLYWHEEL_MOTOR_SIGN = 1;
    /** PLACEHOLDER: its encoder counts down while the wheel spins forward (what Flywheel.ENCODER_REVERSED = true expects). */
    static final double FLYWHEEL_ENCODER_SIGN = -1;
    /** PLACEHOLDER: a full battery, no sag. */
    static final double BATTERY_VOLTS = 12.0;

    /** The sim clock: the runner advances it, so anything that reads it sees sim time. */
    public final FakeTimer timer = new FakeTimer();
    public final FakeHardwareMap hardwareMap = new FakeHardwareMap();

    private final FakeMotor[] drive = new FakeMotor[4];
    private final MecanumDrivePlant drivePlant;
    private final SimMotor flywheel = new SimMotor();
    private final FakeOctoQuad octoQuad = new FakeOctoQuad();
    private final FakeVoltageSensor battery = new FakeVoltageSensor(BATTERY_VOLTS);
    private final FlywheelModel flywheelModel = new FlywheelModel();
    private double lastHeading;

    public BiobuzzSim() {
        for (int i = 0; i < 4; i++) {
            drive[i] = new FakeMotor(MAX_FORWARD_IN_PER_S * TICKS_PER_INCH, DRIVE_MAX_ACCEL, DRIVE_TICKS_PER_REV);
            hardwareMap.register(DRIVE_MOTORS[i], drive[i]);
        }
        drivePlant = new MecanumDrivePlant(drive, new DriveConfig());
        hardwareMap.register(OCTOQUAD, octoQuad);
        hardwareMap.register(VOLTAGE_SENSOR, battery);
        hardwareMap.register(Flywheel.MOTOR_1, flywheel);
        hardwareMap.register(Flywheel.MOTOR_2, new SimMotor());
        hardwareMap.register(Intake.STAGE1, new SimMotor());
        hardwareMap.register(Intake.STAGE2, new SimMotor());
        for (String servo : new String[] {Hood.SERVO, Turret.LEFT_SERVO, Turret.RIGHT_SERVO,
                FlowerArm.LEFT_SERVO, FlowerArm.RIGHT_SERVO}) {
            hardwareMap.register(servo, new SimServo());
        }
    }

    /**
     * Puts the robot at a Pedro pose (inches, radians), before a run. The OctoQuad sees the true pose at once:
     * INIT's reset and setLocalizerPose run inside the first tick's code, before any step.
     */
    public void placeAt(double pedroX, double pedroY, double headingRad) {
        drivePlant.setPose(pedroX - PEDRO_OFFSET_IN, pedroY - PEDRO_OFFSET_IN, headingRad);
        lastHeading = headingRad;
        pushPose(0);
    }

    /** Places the robot at a route export's "start" pose (what the autos start from, through StartPose). */
    public void placeAtStartOf(Object export) {
        Pose start = exportStart(export);
        placeAt(start.x(), start.y(), start.heading());
    }

    /**
     * One tick of the world after the code ran and the runner advanced the fake devices: drive plant, flywheel model,
     * then the OctoQuad readings the code sees next tick. Never calls hardwareMap.updateAll (Trap 5).
     */
    public void step(double dt) {
        drivePlant.update(dt);
        double heading = drivePlant.getHeading();
        double turnRate = dt > 0 ? (heading - lastHeading) / dt : 0;
        lastHeading = heading;
        pushPose(turnRate);

        flywheelModel.step(FLYWHEEL_MOTOR_SIGN * flywheel.hubPower(), battery.getVoltage(), dt);
        octoQuad.setEncoderVelocity(Flywheel.OCTOQUAD_PORT,
                FLYWHEEL_ENCODER_SIGN * flywheelModel.rpm() / 60.0 * Flywheel.COUNTS_PER_REV);
    }

    private void pushPose(double turnRate) {
        octoQuad.setTrueState(drivePlant.getX(), drivePlant.getY(), drivePlant.getHeading(),
                drivePlant.getForwardVelocity(), drivePlant.getLateralVelocity(), turnRate);
    }

    /** True robot x, Pedro frame, inches. */
    public double pedroX() {
        return drivePlant.getX() + PEDRO_OFFSET_IN;
    }

    /** True robot y, Pedro frame, inches. */
    public double pedroY() {
        return drivePlant.getY() + PEDRO_OFFSET_IN;
    }

    /** True heading, radians CCW, not wrapped. */
    public double heading() {
        return drivePlant.getHeading();
    }

    /** True flywheel speed, RPM. */
    public double flywheelRpm() {
        return flywheelModel.rpm();
    }

    public FakeOctoQuad octoQuad() {
        return octoQuad;
    }

    /**
     * A runner for an OpMode adapter class (org.firstinspires.ftc.teamcode.sim), set up the way every sim run is:
     * our code, Pedro, Ivy and NextFTC reloaded onto this sim's clock, the blackboard kept, INIT_LOOP_TICKS then
     * ACTIVE_TICKS, and this world stepped each tick before afterStep. A runner runs once; build a new BiobuzzSim and
     * runner for every run.
     */
    public OpModeRunner runner(Class<? extends OpMode> adapter, final ScenarioRunner.ScenarioBody afterStep) {
        return OpModeRunner.forTeamClass(adapter, hardwareMap)
                .teamPackages(TEAM_PACKAGES)
                .timer(timer)
                .keepBlackboard(true)
                .initLoopTicks(INIT_LOOP_TICKS)
                .activeTicks(ACTIVE_TICKS)
                .eachTick(dt -> {
                    step(dt);
                    afterStep.tick(dt);
                });
    }

    public OpModeRunner runner(Class<? extends OpMode> adapter) {
        return runner(adapter, dt -> { });
    }

    /**
     * A route export's private "start" pose (paths/BlueAutoPaths, RedAutoPaths), read the way StartPose reads it:
     * the AutoRoute adapters inherit the field, so walk up. Red's literal equals blue's; its PoseFactory mirrors it.
     */
    public static Pose exportStart(Object export) {
        for (Class<?> type = export.getClass(); type != null; type = type.getSuperclass()) {
            try {
                Field field = type.getDeclaredField("start");
                field.setAccessible(true);
                return (Pose) field.get(export);
            } catch (NoSuchFieldException e) {
            } catch (IllegalAccessException e) {
                throw new IllegalStateException("Cannot read the export's start pose", e);
            }
        }
        throw new IllegalStateException(export.getClass().getName() + " has no 'start' pose");
    }

    /** Pedro's drive slots, the OctoQuad and the drive plant's numbers (ROBOT_CONFIG). */
    private static final class BiobuzzSimConfig implements SimRobotConfig {
        @Override
        public String[] driveMotorNames() {
            return DRIVE_MOTORS.clone();
        }

        @Override
        public String odometryName() {
            return OCTOQUAD;
        }

        @Override
        public DrivetrainSimConfig drivetrain() {
            return new DriveConfig();
        }
    }

    /** The drive plant's geometry and limits (PLACEHOLDER numbers above). */
    private static final class DriveConfig implements DrivetrainSimConfig {
        @Override
        public double trackWidth() {
            return TRACK_WIDTH_IN;
        }

        @Override
        public double wheelBase() {
            return WHEEL_BASE_IN;
        }

        @Override
        public double wheelRadius() {
            return WHEEL_DIAMETER_IN / 2;
        }

        @Override
        public double ticksPerInch() {
            return TICKS_PER_INCH;
        }

        @Override
        public double maxVelocityTicksPerSecond() {
            return MAX_FORWARD_IN_PER_S * TICKS_PER_INCH;
        }

        @Override
        public double maxLateralVelocityTicksPerSecond() {
            return MAX_STRAFE_IN_PER_S * TICKS_PER_INCH;
        }

        /** Not read by the plant: each FakeMotor gets it in its constructor. */
        @Override
        public double maxAccel() {
            return DRIVE_MAX_ACCEL;
        }

        @Override
        public double naturalForwardDecelerationInchesPerSecondSquared() {
            return FORWARD_COAST_DECEL;
        }

        @Override
        public double naturalLateralDecelerationInchesPerSecondSquared() {
            return STRAFE_COAST_DECEL;
        }

        @Override
        public double maxForwardAccelerationInchesPerSecondSquared() {
            return FORWARD_TRACTION;
        }

        @Override
        public double maxLateralAccelerationInchesPerSecondSquared() {
            return STRAFE_TRACTION;
        }

        @Override
        public double[] wheelMountingSigns() {
            return DRIVE_MOUNTING_SIGNS.clone();
        }

        @Override
        public double fieldHalfWidth() {
            return FIELD_HALF_IN;
        }

        @Override
        public double fieldHalfHeight() {
            return FIELD_HALF_IN;
        }
    }
}
