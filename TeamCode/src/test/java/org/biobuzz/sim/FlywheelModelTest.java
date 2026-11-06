package org.biobuzz.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.shooter.ShotTable;
import org.junit.Test;

/** The flywheel plant: a first-order rise toward the speed the applied volts hold (PLACEHOLDER constants). */
public class FlywheelModelTest {

    private static final double EPS = 1e-9;
    private static final double VOLTS = 12.0;

    @Test
    public void steadySpeedIsWhatTheVoltsHoldPastFriction() {
        double power = 0.6;
        double expected = (power * VOLTS - FlywheelModel.KS_VOLTS) / FlywheelModel.KV_VOLTS_PER_RPM;
        assertTrue(expected > 0);
        assertEquals(expected, FlywheelModel.steadyRpm(power, VOLTS), EPS);
        assertEquals(-expected, FlywheelModel.steadyRpm(-power, VOLTS), EPS);
    }

    @Test
    public void powerBelowFrictionHoldsNoSpeed() {
        assertEquals(0.0, FlywheelModel.steadyRpm(0.99 * FlywheelModel.KS_VOLTS / VOLTS, VOLTS), 0.0);
        assertEquals(0.0, FlywheelModel.steadyRpm(-0.99 * FlywheelModel.KS_VOLTS / VOLTS, VOLTS), 0.0);
        assertEquals(0.0, FlywheelModel.steadyRpm(0.0, VOLTS), 0.0);
    }

    @Test
    public void fullPowerHoldsMoreThanTheShotTableAsks() {
        double fastest = 0;
        for (int i = 0; i < ShotTable.RPM.size(); i++) {
            fastest = Math.max(fastest, ShotTable.RPM.valueAt(i));
        }
        assertTrue("full power holds " + FlywheelModel.steadyRpm(1.0, VOLTS) + " RPM, the table asks for "
                + fastest, FlywheelModel.steadyRpm(1.0, VOLTS) > fastest);
    }

    @Test
    public void startsStillAndRisesSixtyThreePercentInOneTimeConstant() {
        FlywheelModel model = new FlywheelModel();
        assertEquals(0.0, model.rpm(), 0.0);
        model.step(1.0, VOLTS, FlywheelModel.TIME_CONSTANT_S);
        assertEquals(FlywheelModel.steadyRpm(1.0, VOLTS) * (1 - Math.exp(-1)), model.rpm(), 1e-6);
    }

    @Test
    public void fiftyTicksMatchOneSecondStep() {
        FlywheelModel ticked = new FlywheelModel();
        for (int i = 0; i < 50; i++) {
            ticked.step(0.8, VOLTS, 0.02);
        }
        FlywheelModel once = new FlywheelModel();
        once.step(0.8, VOLTS, 1.0);
        assertTrue(once.rpm() > 0);
        assertEquals(once.rpm(), ticked.rpm(), 1e-6);
    }

    @Test
    public void settlesAtTheSteadySpeedThenCoastsDownWithNoPower() {
        FlywheelModel model = new FlywheelModel();
        model.step(0.7, VOLTS, 20 * FlywheelModel.TIME_CONSTANT_S);
        double steady = FlywheelModel.steadyRpm(0.7, VOLTS);
        assertEquals(steady, model.rpm(), 1e-3);
        model.step(0.0, VOLTS, FlywheelModel.TIME_CONSTANT_S);
        assertEquals(steady * Math.exp(-1), model.rpm(), 1e-3);
    }

    @Test
    public void aTickThatIsNotForwardInTimeChangesNothing() {
        FlywheelModel model = new FlywheelModel();
        model.step(1.0, VOLTS, 0.0);
        model.step(1.0, VOLTS, -0.02);
        model.step(1.0, VOLTS, Double.NaN);
        assertEquals(0.0, model.rpm(), 0.0);
    }
}
