package org.biobuzz.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.shooter.Flywheel;
import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.RlogDecodedCompare;
import org.horizon36596.simloop.sim.ScenarioRunner;
import org.junit.Test;
import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Map;

/** SimLog: the frame and units a run logs poses in, its keys, and reading a logged pose back. */
public class SimLogTest {

    private static final double M_PER_IN = 0.0254;
    private static final double EXACT = 1e-12;

    @Test
    public void aPedroPoseIsLoggedInThePlantFrameInMetres() {
        assertPose(0, 0, 0, SimLog.loggedPose(72, 72, 0));
        assertPose(38.30 * M_PER_IN, 61.21 * M_PER_IN, Math.PI / 2, SimLog.loggedPose(110.30, 133.21, Math.PI / 2));
        assertPose(-72 * M_PER_IN, 72 * M_PER_IN, 0, SimLog.loggedPose(0, 144, 0));
    }

    @Test
    public void theHeadingIsWrappedToMinusPiUpToPi() {
        assertEquals(-Math.PI / 2, heading(SimLog.loggedPose(72, 72, 3 * Math.PI / 2)), EXACT);
        assertEquals(-Math.PI, heading(SimLog.loggedPose(72, 72, Math.PI)), EXACT);
        assertEquals(0.5, heading(SimLog.loggedPose(72, 72, 0.5 - 4 * Math.PI)), 1e-9);
    }

    @Test
    public void aPoseReadsBackAsTheRunLogDecoderRendersIt() {
        LogTable table = new LogTable(0);
        table.put("pose", Pose2d.struct, new Pose2d(-1.25, 0.75, new Rotation2d(-2.5)));
        Pose2d decoded = SimLog.decodePose(String.valueOf(table.get("pose")));
        assertEquals(-1.25, decoded.getX(), 0);
        assertEquals(0.75, decoded.getY(), 0);
        assertEquals(-2.5, heading(decoded), 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void aValueThatIsNotAPose2dIsRefused() {
        SimLog.decodePose("[1, 2, 3]");
    }

    @Test
    public void recordPutsEveryKeyInTheRunLog() throws IOException {
        final BiobuzzSim sim = new BiobuzzSim();
        sim.placeAt(110.30, 133.21, Math.PI / 2);
        sim.hardwareMap.get(DcMotorEx.class, Flywheel.MOTOR_1).setPower(1);
        final int[] tick = {0};
        Path rlog = Paths.get("build", "sim", "simlog-test.rlog");
        ScenarioRunner.run("SimLogTest", BiobuzzSim.ROBOT_CONFIG, new FakeTimer(), 3, 0.02, rlog, dt -> {
            sim.step(dt);
            if (tick[0]++ == 0) {
                SimLog.record(sim, null, null, "before start");
            } else {
                SimLog.record(sim, new double[] {100, 30, 4.0}, "B", "volley-B2");
            }
        });

        List<Map<String, String>> frames = RlogDecodedCompare.decode(rlog);
        assertEquals(3, frames.size());
        Map<String, String> first = frames.get(0);
        assertNull(first.get(SimLog.READ_PREFIX + SimLog.BELIEVED_POSE));
        assertNull(first.get(SimLog.READ_PREFIX + SimLog.TARGET_CELL));
        assertEquals("before start", first.get(SimLog.READ_PREFIX + SimLog.STEP));

        Map<String, String> last = frames.get(2);
        for (String key : SimLog.KEYS) {
            assertTrue("no " + key + " in " + last.keySet(), last.containsKey(SimLog.READ_PREFIX + key));
        }
        assertPose(38.30 * M_PER_IN, 61.21 * M_PER_IN, Math.PI / 2,
                SimLog.decodePose(last.get(SimLog.READ_PREFIX + SimLog.TRUE_POSE)));
        assertPose(28 * M_PER_IN, -42 * M_PER_IN, 4.0 - 2 * Math.PI,
                SimLog.decodePose(last.get(SimLog.READ_PREFIX + SimLog.BELIEVED_POSE)));
        assertEquals("B", last.get(SimLog.READ_PREFIX + SimLog.TARGET_CELL));
        assertTrue("the flywheel never spun up: " + sim.flywheelRpm(), sim.flywheelRpm() > 0);
        assertEquals(sim.flywheelRpm(), Double.parseDouble(last.get(SimLog.READ_PREFIX + SimLog.FLYWHEEL_RPM)), 0);
        assertEquals("volley-B2", last.get(SimLog.READ_PREFIX + SimLog.STEP));
    }

    private static double heading(Pose2d pose) {
        return pose.getRotation().getRadians();
    }

    private static void assertPose(double x, double y, double heading, Pose2d actual) {
        assertEquals("x, m", x, actual.getX(), EXACT);
        assertEquals("y, m", y, actual.getY(), EXACT);
        assertEquals("heading, rad", heading, heading(actual), EXACT);
    }
}
