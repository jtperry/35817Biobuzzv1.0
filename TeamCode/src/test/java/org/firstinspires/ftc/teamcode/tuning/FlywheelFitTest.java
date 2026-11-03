package org.firstinspires.ftc.teamcode.tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class FlywheelFitTest {

    private static final double EPS = 1e-9;
    private static final double KS = 1.2;
    private static final double KV = 0.0021;
    private static final double[] RPM = {2500, 2800, 3100, 3400, 3700, 4000};

    private static double[] onLine(double[] rpm) {
        double[] volts = new double[rpm.length];
        for (int i = 0; i < rpm.length; i++) {
            volts[i] = KS + KV * rpm[i];
        }
        return volts;
    }

    private static void assertNoFit(FlywheelFit.Result fit) {
        assertTrue(Double.isNaN(fit.ksVolts));
        assertTrue(Double.isNaN(fit.kvVoltsPerRpm));
        assertTrue(Double.isNaN(fit.rSquared));
    }

    @Test
    public void recoversAKnownLineExactly() {
        FlywheelFit.Result fit = FlywheelFit.fit(RPM, onLine(RPM), RPM.length);
        assertEquals(KS, fit.ksVolts, EPS);
        assertEquals(KV, fit.kvVoltsPerRpm, 1e-12);
        assertEquals(6, fit.samples);
    }

    @Test
    public void perfectLineHasRSquaredOne() {
        FlywheelFit.Result fit = FlywheelFit.fit(RPM, onLine(RPM), RPM.length);
        assertEquals(1.0, fit.rSquared, EPS);
    }

    @Test
    public void noisyDataStaysCloseToTheLine() {
        double[] noise = {0.04, -0.03, 0.02, -0.05, 0.03, -0.01};
        double[] volts = onLine(RPM);
        for (int i = 0; i < volts.length; i++) {
            volts[i] += noise[i];
        }
        FlywheelFit.Result fit = FlywheelFit.fit(RPM, volts, RPM.length);
        // Least squares on these six points: kS 1.2433, kV 0.0020867, r squared 0.9991.
        assertEquals(KS, fit.ksVolts, 0.1);
        assertEquals(KV, fit.kvVoltsPerRpm, 2e-5);
        assertTrue(fit.rSquared > 0.99);
        assertTrue(fit.rSquared < 1.0);
    }

    @Test
    public void ignoresPointsPastCount() {
        double[] rpm = {2500, 3000, 3500, 1, 1};
        double[] volts = onLine(rpm);
        volts[3] = 100;
        volts[4] = -100;
        FlywheelFit.Result fit = FlywheelFit.fit(rpm, volts, 3);
        assertEquals(KS, fit.ksVolts, EPS);
        assertEquals(KV, fit.kvVoltsPerRpm, 1e-12);
        assertEquals(3, fit.samples);
    }

    @Test
    public void fewerThanTwoPointsGiveNoFit() {
        FlywheelFit.Result none = FlywheelFit.fit(new double[0], new double[0], 0);
        assertNoFit(none);
        assertEquals(0, none.samples);
        FlywheelFit.Result one = FlywheelFit.fit(new double[] {3000}, new double[] {7.5}, 1);
        assertNoFit(one);
        assertEquals(1, one.samples);
    }

    @Test
    public void oneDistinctRpmGivesNoFit() {
        // A non-integer RPM repeated: its mean can differ from it by rounding, so only an explicit check catches it.
        double[] rpm = {3123.7, 3123.7, 3123.7};
        double[] volts = {7.0, 7.5, 8.0};
        FlywheelFit.Result fit = FlywheelFit.fit(rpm, volts, 3);
        assertNoFit(fit);
        assertEquals(3, fit.samples);
    }

    @Test
    public void flatVoltsGiveNoRSquared() {
        // 7.1 is not exact in binary, so the mean of three of them is off by rounding: only an explicit check gives NaN.
        double[] rpm = {2500, 3000, 3500};
        double[] volts = {7.1, 7.1, 7.1};
        FlywheelFit.Result fit = FlywheelFit.fit(rpm, volts, 3);
        assertEquals(7.1, fit.ksVolts, EPS);
        assertEquals(0.0, fit.kvVoltsPerRpm, 1e-12);
        assertTrue(Double.isNaN(fit.rSquared));
    }
}
