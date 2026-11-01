package org.firstinspires.ftc.teamcode.shooter;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

public class ShotSolverTest {

    private static final double EPS = 1e-9;
    private static final double FLIGHT_S = 0.5;
    private static final double OFFSET = ShotSolver.TURRET_OFFSET_FORWARD_IN;

    private ShotSolver solver;

    @Before
    public void setUp() {
        ShotSolver.SHOOT_ON_MOVE = true;
        ShotSolver.RELEASE_LATENCY_S = 0.15;
        ShotSolver.LEAD_GAIN = 1.0;
        ShotSolver.ACCEL_SMOOTHING = 1.0;
        ShotSolver.MAX_ACCEL_IN_S2 = 1000;
        ShotSolver.MIN_SPEED_IN_S = 3;
        ShotSolver.RATE_SMOOTHING = 1.0;
        solver = new ShotSolver(distance -> FLIGHT_S);
    }

    @Test
    public void standingStillAimsStraightAtTheCell() {
        ShotSolution shot = solver.update(0, 0, 0, 0, 0, 0, 0.01, 100, 0);
        assertEquals(0.0, shot.turretDeg, EPS);
        assertEquals(100 - OFFSET, shot.distanceIn, EPS);
        assertEquals(100.0, shot.aimX, EPS);
        assertEquals(0.0, shot.aimY, EPS);
    }

    @Test
    public void cellToTheLeftIsPositiveTurretAngle() {
        ShotSolution shot = solver.update(0, 0, 0, 0, 0, 0, 0.01, 0, 100);
        assertEquals(Math.toDegrees(Math.atan2(100, -OFFSET)), shot.turretDeg, EPS);
    }

    @Test
    public void constantVelocityLeadsByFlightTime() {
        solver.update(0, 0, 0, 0, 20, 0, 0.01, 100, 0);
        ShotSolution shot = solver.update(0, 0, 0, 0, 20, 0, 0.01, 100, 0);
        assertEquals(3.0, shot.releaseY, EPS);
        assertEquals(100.0, shot.aimX, EPS);
        assertEquals(-10.0, shot.aimY, EPS);
        assertEquals(20.0, shot.releaseSpeed, EPS);
        assertEquals(Math.toDegrees(Math.atan2(-13, 100 - OFFSET)), shot.turretDeg, EPS);
    }

    @Test
    public void brakingToAStopBeforeReleaseRemovesTheLead() {
        solver.update(0, 0, 0, 0, 40, 0, 0.1, 100, 0);
        ShotSolution shot = solver.update(0, 0, 0, 0, 20, 0, 0.1, 100, 0);
        // -200 in/s^2 stops 20 in/s in 0.1 s (< 0.15 s latency): stopping distance 1 in, no lead.
        assertEquals(0.0, shot.releaseSpeed, EPS);
        assertEquals(1.0, shot.releaseY, EPS);
        assertEquals(100.0, shot.aimX, EPS);
        assertEquals(0.0, shot.aimY, EPS);
    }

    @Test
    public void brakingSlowlyShrinksTheLeadButNeverFlipsIt() {
        solver.update(0, 0, 0, 0, 40, 0, 0.1, 100, 0);
        ShotSolution shot = solver.update(0, 0, 0, 0, 38, 0, 0.1, 100, 0);
        // -20 in/s^2: release speed 38 - 3 = 35, travel (38 + 35) / 2 * 0.15 = 5.475.
        assertEquals(35.0, shot.releaseSpeed, EPS);
        assertEquals(5.475, shot.releaseY, EPS);
        assertEquals(-17.5, shot.aimY, EPS);
    }

    @Test
    public void belowMinimumSpeedThereIsNoLead() {
        ShotSolution shot = solver.update(0, 0, 0, 0, 2, 0, 0.01, 100, 0);
        assertEquals(0.0, shot.aimY, EPS);
        assertEquals(0.0, shot.releaseY, EPS);
    }

    @Test
    public void shootOnMoveOffIgnoresVelocity() {
        ShotSolver.SHOOT_ON_MOVE = false;
        ShotSolution shot = solver.update(0, 0, 0, 0, 20, 0, 0.01, 100, 0);
        assertEquals(0.0, shot.aimY, EPS);
        assertEquals(0.0, shot.releaseY, EPS);
        assertEquals(OFFSET, shot.releaseX, EPS);
    }

    @Test
    public void turningUsesTheHeadingAtRelease() {
        ShotSolution shot = solver.update(0, 0, 0, 0, 0, 1.0, 0.01, 100, 0);
        double heading = 0.15;
        double rx = OFFSET * Math.cos(heading);
        double ry = OFFSET * Math.sin(heading);
        assertEquals(Math.toDegrees(Math.atan2(-ry, 100 - rx) - heading), shot.turretDeg, EPS);
    }

    @Test
    public void turretRateFollowsAngleChange() {
        ShotSolution first = solver.update(0, 0, 0, 0, 0, 0, 0.1, 100, 0);
        double firstDeg = first.turretDeg;
        ShotSolution second = solver.update(0, 0, 0.1, 0, 0, 0, 0.1, 100, 0);
        assertEquals((second.turretDeg - firstDeg) / 0.1, second.turretRateDegPerS, EPS);
    }
}
