package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.math.Pose;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.shooter.Flywheel;

import java.lang.reflect.Field;
import java.util.function.BiConsumer;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.OpModeHook;

/**
 * Runs a NextOpMode in SimLoop the way NextFTC runs it on the robot. SimLoop needs a LinearOpMode with a no-arg
 * constructor; NextFTC's own BoundNextOpMode takes the OpMode's constructor function, so this copies its runOpMode,
 * adding one read-only step after each loop pass: reportBelief.
 *
 * <p>This is a team-package class: SimLoop loads it, with our code, Pedro, Ivy and NextFTC, fresh for every run, so
 * every static it sets (RobotController.eventLoop, Flywheel.AT_SPEED_DEBOUNCE_S) is that run's own (Trap 1).
 */
public abstract class NextOpModeSim extends LinearOpMode {

    /**
     * Where a sim run listens for what the code believes: a {@code java.util.function.BiConsumer<double[], String>}
     * in OpMode.blackboard under this key. After every INIT and active loop pass it gets the follower's pose
     * {x in, y in, heading rad} (Pedro frame) and the name of the cell the robot aims at: JDK types, because Pedro's
     * Pose and our HiveCell are reloaded per run and would not cast on the test's side. With no sink, it is one
     * lookup a loop.
     */
    public static final String BELIEF_SINK_KEY = "biobuzz.sim.belief";

    /** The NextOpMode under test, built on this run's robot (BoundNextOpMode's constructor function). */
    protected abstract NextOpMode create(Robot robot);

    @Override
    public void runOpMode() {
        RobotController.eventLoop = eventLoopServing(hardwareMap);
        Flywheel.AT_SPEED_DEBOUNCE_S = 0;
        Robot robot = new Robot();

        NextOpMode.Companion.setActiveGamepad1$robot_release(gamepad1);
        NextOpMode.Companion.setActiveGamepad2$robot_release(gamepad2);
        NextOpMode.Companion.setActiveTelemetry$robot_release(telemetry);
        NextOpMode.Companion.setActiveHardwareMap$robot_release(hardwareMap);

        NextOpMode opMode = null;
        try {
            opMode = create(robot);

            for (OpModeHook hook : opMode.getHooks$robot_release()) {
                hook.afterConstruction();
            }
            while (opModeInInit()) {
                for (OpModeHook hook : opMode.getHooks$robot_release()) {
                    hook.beforeDisabled();
                }
                opMode.disabledPeriodic();
                for (OpModeHook hook : opMode.getHooks$robot_release()) {
                    hook.afterDisabled();
                }
                reportBelief(robot);
            }
            waitForStart();
            for (OpModeHook hook : opMode.getHooks$robot_release()) {
                hook.beforeStart();
            }
            opMode.start();
            for (OpModeHook hook : opMode.getHooks$robot_release()) {
                hook.afterStart();
            }
            while (opModeIsActive()) {
                for (OpModeHook hook : opMode.getHooks$robot_release()) {
                    hook.beforePeriodic();
                }
                opMode.periodic();
                for (OpModeHook hook : opMode.getHooks$robot_release()) {
                    hook.afterPeriodic();
                }
                reportBelief(robot);
            }
            for (OpModeHook hook : opMode.getHooks$robot_release()) {
                hook.beforeEnd();
            }
            opMode.end();
        } finally {
            if (opMode != null) {
                for (OpModeHook hook : opMode.getHooks$robot_release()) {
                    try {
                        hook.afterEnd();
                    } catch (Throwable throwable) {
                        System.err.println(hook.getClass().getSimpleName()
                                + " threw while cleaning up after the OpMode.");
                        throwable.printStackTrace();
                    }
                }
            }
            NextOpMode.Companion.setActiveGamepad1$robot_release(null);
            NextOpMode.Companion.setActiveGamepad2$robot_release(null);
            NextOpMode.Companion.setActiveTelemetry$robot_release(null);
            NextOpMode.Companion.setActiveHardwareMap$robot_release(null);
        }
    }

    /**
     * Not in BoundNextOpMode: hands the sim's belief sink (BELIEF_SINK_KEY), if there is one, the pose and cell the
     * code holds after this loop pass. Reads only. In the active loop this follows RobotHook's follower update, so the
     * pose is this loop's OctoQuad read; in INIT the follower is not updated and holds the start pose.
     */
    @SuppressWarnings("unchecked")
    private static void reportBelief(Robot robot) {
        Object sink = OpMode.blackboard.get(BELIEF_SINK_KEY);
        if (!(sink instanceof BiConsumer) || robot.follower() == null) {
            return;
        }
        Pose pose = robot.pose();
        ((BiConsumer<double[], String>) sink).accept(new double[] {pose.x(), pose.y(), pose.heading()},
                robot.cell().name());
    }

    /**
     * An FtcEventLoop whose op mode manager holds this hardware map: RobotController.hardwareMap() reads
     * eventLoop.getOpModeManager().getHardwareMap(). Both are allocated without their constructors, which need the
     * robot app; the loop answers getOpModeManager() and nothing else.
     */
    private static FtcEventLoop eventLoopServing(HardwareMap hardwareMap) {
        OpModeManagerImpl manager = allocateWithoutConstructor(OpModeManagerImpl.class);
        manager.setHardwareMap(hardwareMap);
        SimEventLoop loop = allocateWithoutConstructor(SimEventLoop.class);
        loop.manager = manager;
        return loop;
    }

    /** Unsafe.allocateInstance, by reflection (sun.misc is not on the Android compile classpath). */
    private static <T> T allocateWithoutConstructor(Class<T> type) {
        try {
            Class<?> unsafeType = Class.forName("sun.misc.Unsafe");
            Field theUnsafe = unsafeType.getDeclaredField("theUnsafe");
            theUnsafe.setAccessible(true);
            Object unsafe = theUnsafe.get(null);
            return type.cast(unsafeType.getMethod("allocateInstance", Class.class).invoke(unsafe, type));
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("Cannot allocate " + type.getName() + " without its constructor", e);
        }
    }

    /** Allocated without a constructor, so its field is set after allocation and nothing here is final. */
    private static final class SimEventLoop extends FtcEventLoop {

        OpModeManagerImpl manager;

        /** Never runs: allocateWithoutConstructor skips it. */
        private SimEventLoop() {
            super(null, null, null, null);
        }

        @Override
        public OpModeManagerImpl getOpModeManager() {
            return manager;
        }
    }
}
