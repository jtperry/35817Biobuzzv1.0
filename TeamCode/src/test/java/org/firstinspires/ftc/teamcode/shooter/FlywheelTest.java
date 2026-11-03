package org.firstinspires.ftc.teamcode.shooter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import org.firstinspires.ftc.teamcode.util.Battery;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Field;

public class FlywheelTest {

    private static final double EPS = 1e-9;

    private Flywheel flywheel;
    private double target;

    @Before
    public void setUp() {
        Flywheel.USE_MOTOR_2 = false;
        Flywheel.KI_VOLTS = 0.0;
        Flywheel.I_MAX_VOLTS = 1.5;
        Flywheel.BANG_BANG_RPM = 100;
        Flywheel.RPM_TOLERANCE = 60;
        Flywheel.AT_SPEED_DEBOUNCE_S = 0.1;
        flywheel = new Flywheel(new Battery());
        target = 50;
        flywheel.fixed(() -> target);
    }

    /** The controller state has no getter; the tests read it by name. */
    private double state(String name) throws Exception {
        Field field = Flywheel.class.getDeclaredField(name);
        field.setAccessible(true);
        return field.getDouble(flywheel);
    }

    @Test
    public void nanFixedTargetCountsAsZero() {
        target = Double.NaN;
        flywheel.periodic();
        assertEquals(0.0, flywheel.targetRpm(), EPS);
        assertFalse(flywheel.atSpeed());
    }

    @Test
    public void infiniteFixedTargetCountsAsZero() {
        target = Double.POSITIVE_INFINITY;
        flywheel.periodic();
        assertEquals(0.0, flywheel.targetRpm(), EPS);
    }

    @Test
    public void nanTrackingTargetCountsAsZero() {
        flywheel.arm();
        flywheel.setTrackingRpm(Double.NaN);
        flywheel.periodic();
        assertEquals(0.0, flywheel.targetRpm(), EPS);
    }

    @Test
    public void nanTargetClearsIntegralAndLastError() throws Exception {
        flywheel.periodic();
        assertEquals(50.0, flywheel.targetRpm(), EPS);
        assertEquals(50.0, state("lastError"), EPS);
        target = Double.NaN;
        flywheel.periodic();
        assertEquals(0.0, state("integral"), EPS);
        assertEquals(0.0, state("lastError"), EPS);
    }

    @Test
    public void controlResumesWhenTheTargetIsFiniteAgain() throws Exception {
        target = Double.NaN;
        flywheel.periodic();
        flywheel.periodic();
        target = 50;
        flywheel.periodic();
        flywheel.periodic();
        assertEquals(50.0, flywheel.targetRpm(), EPS);
        assertFalse(Double.isNaN(state("integral")));
        assertEquals(50.0, state("lastError"), EPS);
    }
}
