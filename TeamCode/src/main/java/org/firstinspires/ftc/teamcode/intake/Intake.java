package org.firstinspires.ftc.teamcode.intake;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

/**
 * Two intake stages, no gate: while intaking, stage 2 runs backwards so balls wait before the shooter. Command-based:
 * its commands require it. Positive power = toward the shooter.
 */
@Configurable
public class Intake implements Mechanism {

    public static final String STAGE1 = "stage1";
    public static final String STAGE2 = "stage2";
    /** Motor sign that moves each stage toward the shooter (mounted opposite). Flip live during bring-up. */
    public static double STAGE1_SIGN = -1.0;
    public static double STAGE2_SIGN = 1.0;

    public static double STAGE1_INTAKE_POWER = 1.0;
    /** Stage 2 runs backwards while intaking so balls cannot reach the shooter. */
    public static double STAGE2_INTAKE_POWER = -0.5;
    public static double STAGE1_FEED_POWER = 0.8;
    public static double STAGE2_FEED_POWER = 1.0;
    public static double EJECT_POWER = -1.0;

    private NextMotor stage1;
    private NextMotor stage2;
    private double stage1Power = 0;
    private double stage2Power = 0;

    public Intake() {
        bindMotors();
    }

    /** Fresh motor objects each OpMode (Robot.bindHardware), for the same reason as Flywheel.bindMotors(). */
    public void bindMotors() {
        stage1 = new NextMotor(STAGE1);
        stage2 = new NextMotor(STAGE2);
    }

    /** Known state: stages off. */
    public void reset() {
        runStages(0, 0);
        stage1.update();
        stage2.update();
    }

    @Override
    public Command getDefaultCommand() {
        return stopRollers();
    }


    public Command intake() {
        return infinite(() -> runStages(STAGE1_INTAKE_POWER, STAGE2_INTAKE_POWER));
    }

    public Command eject() {
        return infinite(() -> runStages(EJECT_POWER, EJECT_POWER));
    }

    /** Default: stages off. */
    public Command stopRollers() {
        return infinite(() -> runStages(0, 0));
    }

    /** End of a shot: stages off. */
    public Command stopFeed() {
        return instant(() -> runStages(0, 0));
    }

    public Command stopIntake() {
        return instant(() -> runStages(0, 0));
    }

    /**
     * The stages push balls into the shooter while feeding is true, else they stop. However it ends (done, cancelled,
     * overridden by intake/eject, a race or deadline parent): stages off.
     */
    public Command feed(BooleanSupplier feeding) {
        return infinite(() -> {
            if (feeding.getAsBoolean()) {
                runStages(STAGE1_FEED_POWER, STAGE2_FEED_POWER);
            } else {
                runStages(0, 0);
            }
        }).setEnd(endCondition -> runStages(0, 0));
    }

    /** Raw stage powers (tests). */
    public Command manual(DoubleSupplier stage1Power, DoubleSupplier stage2Power) {
        return infinite(() -> runStages(stage1Power.getAsDouble(), stage2Power.getAsDouble()));
    }


    /** Last stage 1 power asked for, before the sign (tests). */
    double stage1Power() {
        return stage1Power;
    }

    /** Last stage 2 power asked for, before the sign (tests). */
    double stage2Power() {
        return stage2Power;
    }

    /** Separate hub read each call: tests only. */
    public double stage1Amps() {
        return stage1.getCurrent().into(Units.Amperes);
    }

    public double stage2Amps() {
        return stage2.getCurrent().into(Units.Amperes);
    }


    private void runStages(double stage1Power, double stage2Power) {
        this.stage1Power = stage1Power;
        this.stage2Power = stage2Power;
        stage1.setThrottle(MathUtil.safePower(stage1Power * STAGE1_SIGN));
        stage2.setThrottle(MathUtil.safePower(stage2Power * STAGE2_SIGN));
    }
}
