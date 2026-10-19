package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.globals.Constants;

import java.util.function.DoubleSupplier;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

/** Intake. Two stages, gate, flower arm. */
public class IntakeTransfer implements Mechanism {

    private final NextMotor stage1 = new NextMotor("stage1");
    private final NextMotor stage2 = new NextMotor("stage2");
    // Not final. bindServos() replace each OpMode.
    private NextServo gate;
    private NextServo flowerLeft;
    private NextServo flowerRight;

    private boolean gateOpen = false;
    private boolean flowerDeployed = false;

    public IntakeTransfer() {
        bindServos();
    }

    /** Make fresh gate + flower servos. Call every OpMode start. */
    public void bindServos() {
        gate = new NextServo("gate");
        flowerLeft = new NextServo("flowerLeft");
        flowerRight = new NextServo("flowerRight");
        flowerRight.setDirection(NextMotor.Direction.REVERSE);
        // SDK reset range each OpMode. Set again here.
        flowerLeft.setPwmRange(Constants.FLOWER_SERVO_PWM_MIN_US, Constants.FLOWER_SERVO_PWM_MAX_US);
        flowerRight.setPwmRange(Constants.FLOWER_SERVO_PWM_MIN_US, Constants.FLOWER_SERVO_PWM_MAX_US);
    }

    // ---------------------------------------------------------------- lifecycle

    /** Known state. Motors off, gate closed, arm stowed. Flags match hardware. */
    public void reset() {
        runStages(0);
        applyGate(false);
        applyFlower(false);
    }

    @Override
    public Command getDefaultCommand() {
        return stopRollers();
    }

    // ---------------------------------------------------------------- state

    public boolean isGateOpen() {
        return gateOpen;
    }

    public boolean isFlowerDeployed() {
        return flowerDeployed;
    }

    /** Stage 1 amps. */
    public double stage1CurrentAmps() {
        return stage1.getCurrent().into(Units.Amperes);
    }

    /** Stage 2 amps. */
    public double stage2CurrentAmps() {
        return stage2.getCurrent().into(Units.Amperes);
    }

    // ---------------------------------------------------------------- commands

    /** Stage 1 in, stage 2 half speed backwards. Gate closed. */
    public Command intake() {
        return infinite(() -> {
            setStages(Constants.STAGE1_INTAKE_POWER, Constants.STAGE2_INTAKE_POWER);
        });
    }

    /** Both stages reverse. Clear jam. */
    public Command eject() {
        return infinite(() -> runStages(Constants.EJECT_POWER));
    }

    /** Default command. Motors off only, never touch servos. */
    public Command stopRollers() {
        return infinite(() -> runStages(0));
    }

    /** Raw control for testing. Stage powers, gate + flower servo position. Null = leave alone. */
    public Command manualControl(
            DoubleSupplier stage1Power, DoubleSupplier stage2Power,
            DoubleSupplier gatePosition, DoubleSupplier flowerPosition) {
        return infinite(() -> {
            if (stage1Power != null) {
                setStage1(stage1Power.getAsDouble());
            }
            if (stage2Power != null) {
                setStage2(stage2Power.getAsDouble());
            }
            if (gatePosition != null) {
                gate.setPosition(Constants.clamp(gatePosition.getAsDouble(), 0, 1));
            }
            if (flowerPosition != null) {
                double flower = Constants.clamp(flowerPosition.getAsDouble(), 0, 1);
                // flowerRight reversed in bindServos(). Both get same position.
                flowerLeft.setPosition(flower);
                flowerRight.setPosition(flower);
            }
        });
    }

    /** Raw stage 1 + stage 2 power. For testing. Gate untouched. */
    public Command rollersTo(DoubleSupplier stage1Power, DoubleSupplier stage2Power) {
        return infinite(() -> {
            setStages(stage1Power.getAsDouble(), stage2Power.getAsDouble());
        });
    }

    /** Raw gate servo position. For tuning. */
    public Command gateTo(DoubleSupplier position) {
        return infinite(() -> gate.setPosition(Constants.clamp(position.getAsDouble(), 0, 1)));
    }

    public Command openGate() {
        return instant(() -> applyGate(true));
    }

    public Command closeGate() {
        return instant(() -> applyGate(false));
    }

    /** Gate open, wait GATE_OPEN_DELAY_MS, then both stages push artifacts into flywheel. */
    public Command feed() {
        long[] startMs = new long[1];
        return infinite(() -> {
            applyGate(true);
            boolean gateClear =
                    System.currentTimeMillis() - startMs[0] >= Constants.GATE_OPEN_DELAY_MS;
            if (gateClear) {
                setStages(Constants.STAGE1_FEED_POWER, Constants.STAGE2_FEED_POWER);
            } else {
                runStages(0);
            }
        }).setStart(() -> {
            startMs[0] = System.currentTimeMillis();
            applyGate(true);
        });
    }

    /** Stop both stages, gate untouched. */
    public Command stopIntake() {
        return instant(() -> {
            runStages(0);
        });
    }

    /** Close gate, stop stages. End of shot. */
    public Command stopFeed() {
        return instant(() -> {
            applyGate(false);
            runStages(0);
        });
    }

    public Command deployFlower() {
        return instant(() -> applyFlower(true));
    }

    public Command stowFlower() {
        return instant(() -> applyFlower(false));
    }

    public Command toggleFlower() {
        return instant(() -> applyFlower(!flowerDeployed));
    }

    // ---------------------------------------------------------------- hardware

    /** Both stages, same power. Positive = toward shooter. */
    private void runStages(double power) {
        setStages(power, power);
    }

    /** Each stage own power. Positive = toward shooter. */
    private void setStages(double stage1Power, double stage2Power) {
        setStage1(stage1Power);
        setStage2(stage2Power);
    }

    // Stages mounted opposite. Only place motor sign applied.
    private void setStage1(double power) {
        stage1.setThrottle(Constants.clamp(power, -1.0, 1.0) * Constants.STAGE1_SIGN);
    }

    private void setStage2(double power) {
        stage2.setThrottle(Constants.clamp(power, -1.0, 1.0) * Constants.STAGE2_SIGN);
    }

    private void applyGate(boolean open) {
        gateOpen = open;
        gate.setPosition(open ? Constants.GATE_OPEN_POSITION : Constants.GATE_CLOSED_POSITION);
    }

    private void applyFlower(boolean deployed) {
        flowerDeployed = deployed;
        double position = deployed
                ? Constants.FLOWER_DEPLOYED_POSITION
                : Constants.FLOWER_STOWED_POSITION;

        // flowerRight reversed in bindServos(). Both get same position.
        flowerLeft.setPosition(position);
        flowerRight.setPosition(position);
    }
}
