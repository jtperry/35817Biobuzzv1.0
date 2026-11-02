package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class MathUtilTest {

    private static final double EPS = 1e-9;

    @Test
    public void clampKeepsValueInsideRange() {
        assertEquals(5.0, MathUtil.clamp(5.0, 0.0, 10.0), EPS);
        assertEquals(0.0, MathUtil.clamp(-3.0, 0.0, 10.0), EPS);
        assertEquals(10.0, MathUtil.clamp(12.0, 0.0, 10.0), EPS);
    }

    @Test
    public void safePowerClampsAndRejectsNaN() {
        assertEquals(1.0, MathUtil.safePower(1.7), EPS);
        assertEquals(-1.0, MathUtil.safePower(-4.0), EPS);
        assertEquals(0.25, MathUtil.safePower(0.25), EPS);
        assertEquals(0.0, MathUtil.safePower(Double.NaN), EPS);
    }

    @Test
    public void wrapRadiansLandsInMinusPiToPi() {
        assertEquals(-Math.PI / 2, MathUtil.wrapRadians(3 * Math.PI / 2), EPS);
        assertEquals(Math.PI / 2, MathUtil.wrapRadians(-3 * Math.PI / 2), EPS);
        assertEquals(-Math.PI, MathUtil.wrapRadians(Math.PI), EPS);
        assertEquals(0.0, MathUtil.wrapRadians(4 * Math.PI), EPS);
    }

    @Test
    public void wrapDegreesLandsInMinus180To180() {
        assertEquals(-170.0, MathUtil.wrapDegrees(190.0), EPS);
        assertEquals(170.0, MathUtil.wrapDegrees(-190.0), EPS);
        assertEquals(-180.0, MathUtil.wrapDegrees(180.0), EPS);
        assertEquals(-180.0, MathUtil.wrapDegrees(-180.0), EPS);
        assertEquals(10.0, MathUtil.wrapDegrees(730.0), EPS);
    }
}
