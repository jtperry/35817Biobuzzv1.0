package org.firstinspires.ftc.teamcode.intake;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

/** Two intake stages and the gate. Command-based: its commands require it. Positive power = toward the shooter. */
@Configurable
public class Intake implements Mechanism {

    public static final String STAGE1 = "stage1";
    public static final String STAGE2 = "stage2";
    public static final String GATE = "gate";
    /** Motor sign that moves each stage toward the shooter (mounted opposite). Flip live during bring-up. */
    public static double STAGE1_SIGN = -1.0;
    public static double STAGE2_SIGN = 1.0;

    public static double STAGE1_INTAKE_POWER = 1.0;
    /** Stage 2 runs backwards while intaking so balls wait at the gate. */
    public static double STAGE2_INTAKE_POWER = -0.5;
    public static double STAGE1_FEED_POWER = 0.8;
    public static double STAGE2_FEED_POWER = 1.0;
    public static double EJECT_POWER = -1.0;

    public static double GATE_OPEN_POSITION = 0.42;
    public static double GATE_CLOSED_POSITION = 0.23;
    /** Stages wait this long after the gate opens. */
    public static double GATE_OPEN_DELAY_MS = 100;

    private NextMotor stage1;
    private NextMotor stage2;
    private NextServo gate;
    private boolean gateOpen = false;
    /** Gate positions go to the servo. A unit test swaps this to run without hardware. */
    DoubleConsumer gateOutput = position -> gate.setPosition(position);

    public Intake() {
        bindMotors();
        bindServos();
    }

    /** Fresh motor objects each OpMode (Robot.bindHardware), for the same reason as Flywheel.bindMotors(). */
    public void bindMotors() {
        stage1 = new NextMotor(STAGE1);
        stage2 = new NextMotor(STAGE2);
    }

    /** Fresh servo object each OpMode. */
    public void bindServos() {
        gate = new NextServo(GATE);
    }

    /** Known state: stages off, gate closed. */
    public void reset() {
        runStages(0, 0);
        // Send the 0 now, like Flywheel.reset(), so the hub and NextMotor's write cache agree from here on.
        stage1.update();
        stage2.update();
        setGate(false);
    }

    @Override
    public Command getDefaultCommand() {
        return stopRollers();
    }

    // ---------------------------------------------------------------- commands

    public Command intake() {
        return infinite(() -> runStages(STAGE1_INTAKE_POWER, STAGE2_INTAKE_POWER));
    }

    public Command eject() {
        return infinite(() -> runStages(EJECT_POWER, EJECT_POWER));
    }

    /** Default: stages off, gate untouched. */
    public Command stopRollers() {
        return infinite(() -> runStages(0, 0));
    }

    /** End of a shot: gate closed, stages off. */
    public Command stopFeed() {
        return instant(() -> {
            setGate(false);
            runStages(0, 0);
        });
    }

    public Command stopIntake() {
        return instant(() -> runStages(0, 0));
    }

    /**
     * Gate open; after GATE_OPEN_DELAY_MS the stages push balls while feeding is true.
     * However it ends (done, cancelled, overridden by intake/eject, a race or deadline parent): gate closed, stages off.
     */
    public Command feed(BooleanSupplier feeding) {
        final long[] openedNanos = {0};
        return infinite(() -> {
            setGate(true);
            boolean gateClear = (System.nanoTime() - openedNanos[0]) / 1e6 >= GATE_OPEN_DELAY_MS;
            if (gateClear && feeding.getAsBoolean()) {
                runStages(STAGE1_FEED_POWER, STAGE2_FEED_POWER);
            } else {
                runStages(0, 0);
            }
        }).setStart(() -> {
            openedNanos[0] = System.nanoTime();
            setGate(true);
        }).setEnd(endCondition -> {
            setGate(false);
            runStages(0, 0);
        });
    }

    /** Raw stage powers and gate (tests). */
    public Command manual(DoubleSupplier stage1Power, DoubleSupplier stage2Power, BooleanSupplier open) {
        return infinite(() -> {
            runStages(stage1Power.getAsDouble(), stage2Power.getAsDouble());
            setGate(open.getAsBoolean());
        });
    }

    // ---------------------------------------------------------------- state

    public boolean isGateOpen() {
        return gateOpen;
    }

    /** Separate hub read each call: tests only. */
    public double stage1Amps() {
        return stage1.getCurrent().into(Units.Amperes);
    }

    public double stage2Amps() {
        return stage2.getCurrent().into(Units.Amperes);
    }

    // ---------------------------------------------------------------- hardware

    private void runStages(double stage1Power, double stage2Power) {
        stage1.setThrottle(MathUtil.safePower(stage1Power * STAGE1_SIGN));
        stage2.setThrottle(MathUtil.safePower(stage2Power * STAGE2_SIGN));
    }

    private void setGate(boolean open) {
        gateOpen = open;
        gateOutput.accept(open ? GATE_OPEN_POSITION : GATE_CLOSED_POSITION);
    }
}
