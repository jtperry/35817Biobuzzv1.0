package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.shooter.Flywheel;

import java.lang.reflect.Field;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.OpModeHook;

/**
 * Runs a NextOpMode in SimLoop the way NextFTC runs it on the robot. SimLoop needs a LinearOpMode with a no-arg
 * constructor; NextFTC's own BoundNextOpMode takes the OpMode's constructor function, so this copies its runOpMode.
 *
 * <p>This is a team-package class: SimLoop loads it, with our code, Pedro, Ivy and NextFTC, fresh for every run, so
 * every static it sets (RobotController.eventLoop, Flywheel.AT_SPEED_DEBOUNCE_S) is that run's own (Trap 1).
 */
public abstract class NextOpModeSim extends LinearOpMode {

    /** The NextOpMode under test, built on this run's robot (BoundNextOpMode's constructor function). */
    protected abstract NextOpMode create(Robot robot);

    @Override
    public void runOpMode() {
        // NextMotor and NextServo find their devices through RobotController.hardwareMap(), which the robot's event
        // loop provides: give it one that serves this OpMode's (fake) hardware map.
        RobotController.eventLoop = eventLoopServing(hardwareMap);
        // NextFTC's Debouncer reads Kotlin's TimeSource, which SimLoop never moves onto sim time. With no debounce,
        // atSpeed follows the RPM check alone and the run stays deterministic; it rises about 0.1 s (5 ticks)
        // earlier than on the robot (decision 42).
        Flywheel.AT_SPEED_DEBOUNCE_S = 0;
        // NextFTC builds the robot once, before any OpMode runs.
        Robot robot = new Robot();

        // BoundNextOpMode.runOpMode from dev.nextftc.v2:robot:0.2.1, step for step (hook order checked with javap).
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
                        // NextFTC logs this through its RobotLog and carries on; the sim prints it.
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
