package org.biobuzz.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.junit.Test;

/** SimMotor: the SDK's own DcMotorImplEx over a fake hub, which is what NextMotor casts its device to. */
public class SimMotorTest {

    private static final double EPS = 1e-12;

    @Test
    public void powerReachesTheHub() {
        SimMotor motor = new SimMotor();
        motor.setPower(0.6);
        assertEquals(0.6, motor.hubPower(), EPS);
        motor.setPower(-0.25);
        assertEquals(-0.25, motor.hubPower(), EPS);
    }

    @Test
    public void reverseDirectionFlipsWhatTheHubIsSent() {
        SimMotor motor = new SimMotor();
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setPower(0.6);
        assertEquals(-0.6, motor.hubPower(), EPS);
        // As the SDK reports it: the command the code gave, direction undone.
        assertEquals(0.6, motor.getPower(), EPS);
    }

    @Test
    public void zeroPowerBehaviorReachesTheHub() {
        SimMotor motor = new SimMotor();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, motor.hubZeroPowerBehavior());
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        assertEquals(DcMotor.ZeroPowerBehavior.FLOAT, motor.getZeroPowerBehavior());
    }

    @Test
    public void currentAndModeCallsDoNotThrow() {
        SimMotor motor = new SimMotor();
        // No current model: Hardware Test reads 0 A in the sim.
        assertEquals(0.0, motor.getCurrent(CurrentUnit.AMPS), 0.0);
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motor.getMode());
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.getMode());
        assertEquals("SimMotor", motor.getDeviceName());
        assertTrue(motor.getConnectionInfo().contains("sim"));
    }

    @Test
    public void foundByNameAsTheTypesOurCodeAsksFor() {
        SimMotor motor = new SimMotor();
        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        hardwareMap.register("flywheel", motor);
        // NextMotor: RobotController.hardwareMap[name] as DcMotorImplEx.
        assertSame(motor, (DcMotorImplEx) hardwareMap.get("flywheel"));
        assertSame(motor, hardwareMap.get(DcMotorEx.class, "flywheel"));
        assertSame(motor, hardwareMap.dcMotor.get("flywheel"));
    }

    @Test
    public void callsTheSimDoesNotModelThrowWithTheirName() {
        SimMotor motor = new SimMotor();
        try {
            motor.getCurrentPosition();
            fail("the encoder count is not simulated");
        } catch (UnsupportedOperationException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("getMotorCurrentPosition"));
        }
        try {
            motor.setVelocity(100);
            fail("hub velocity control is not simulated");
        } catch (UnsupportedOperationException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("setMotorVelocity"));
        }
    }
}
