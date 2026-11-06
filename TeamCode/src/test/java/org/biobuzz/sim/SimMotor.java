package org.biobuzz.sim;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.horizon36596.simloop.fakehardware.FakeDevice;

/**
 * A NextMotor's device: the SDK's own DcMotorImplEx (NextMotor casts to it) over a fake hub, so direction and the
 * power arithmetic are the SDK's. The hub keeps what it was sent; a model reads it (BiobuzzSim: the flywheel).
 * Calls our code never makes throw UnsupportedOperationException with the method name.
 */
public final class SimMotor extends DcMotorImplEx implements FakeDevice {

    public static final int PORT = 0;

    private final Hub hub;

    public SimMotor() {
        this(new Hub());
    }

    private SimMotor(Hub hub) {
        // The 4-argument constructor: the shorter ones look up the "Motor" configuration type, which only the
        // robot app registers (a ClassCastException on a JVM).
        super(hub, PORT, DcMotorSimple.Direction.FORWARD, new MotorConfigurationType());
        this.hub = hub;
    }

    /** The power the hub was last sent, direction applied (what drives the windings). */
    public double hubPower() {
        return hub.power;
    }

    public DcMotor.ZeroPowerBehavior hubZeroPowerBehavior() {
        return hub.zeroPowerBehavior;
    }

    /** The SDK's version reads an Android resource. */
    @Override
    public String getDeviceName() {
        return "SimMotor";
    }

    /** Nothing of its own to advance: BiobuzzSim steps the model that reads hubPower(). */
    @Override
    public void update(double deltaTime) {
    }

    /**
     * One port of a fake Lynx motor controller. A static class: DcMotorImpl's constructor calls setMotorType on it
     * before SimMotor's own fields exist.
     */
    static final class Hub implements DcMotorControllerEx {

        private MotorConfigurationType motorType;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        private double power = 0;
        /** Floating, as the SDK leaves every hub motor before an OpMode (LynxDcMotorController.floatHardware). */
        private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        private boolean enabled = true;

        private static UnsupportedOperationException notSimulated(String method) {
            return new UnsupportedOperationException("SimMotor hub: " + method + " is not simulated");
        }

        // ---- what our code reaches, and the plain reads of that state

        @Override
        public void setMotorType(int motor, MotorConfigurationType motorType) {
            this.motorType = motorType;
        }

        @Override
        public MotorConfigurationType getMotorType(int motor) {
            return motorType;
        }

        /** Stored only: the hub's closed-loop modes are not simulated (our code runs every motor open loop). */
        @Override
        public void setMotorMode(int motor, DcMotor.RunMode mode) {
            this.mode = mode;
        }

        @Override
        public DcMotor.RunMode getMotorMode(int motor) {
            return mode;
        }

        @Override
        public void setMotorPower(int motor, double power) {
            this.power = power;
        }

        @Override
        public double getMotorPower(int motor) {
            return power;
        }

        @Override
        public void setMotorZeroPowerBehavior(int motor, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
            this.zeroPowerBehavior = zeroPowerBehavior;
        }

        @Override
        public DcMotor.ZeroPowerBehavior getMotorZeroPowerBehavior(int motor) {
            return zeroPowerBehavior;
        }

        @Override
        public boolean getMotorPowerFloat(int motor) {
            return zeroPowerBehavior == DcMotor.ZeroPowerBehavior.FLOAT && power == 0;
        }

        @Override
        public void setMotorEnable(int motor) {
            enabled = true;
        }

        @Override
        public void setMotorDisable(int motor) {
            enabled = false;
        }

        @Override
        public boolean isMotorEnabled(int motor) {
            return enabled;
        }

        /** No current model: 0 A (only Hardware Test reads it). */
        @Override
        public double getMotorCurrent(int motor, CurrentUnit unit) {
            return 0;
        }

        // ---- HardwareDevice

        @Override
        public HardwareDevice.Manufacturer getManufacturer() {
            return HardwareDevice.Manufacturer.Other;
        }

        @Override
        public String getDeviceName() {
            return "SimMotor hub";
        }

        @Override
        public String getConnectionInfo() {
            return "sim";
        }

        @Override
        public int getVersion() {
            return 1;
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {
        }

        @Override
        public void close() {
        }

        // ---- not simulated

        @Override
        public boolean isBusy(int motor) {
            throw notSimulated("isBusy");
        }

        @Override
        public void setMotorTargetPosition(int motor, int position) {
            throw notSimulated("setMotorTargetPosition");
        }

        @Override
        public int getMotorTargetPosition(int motor) {
            throw notSimulated("getMotorTargetPosition");
        }

        @Override
        public int getMotorCurrentPosition(int motor) {
            throw notSimulated("getMotorCurrentPosition");
        }

        @Override
        public void resetDeviceConfigurationForOpMode(int motor) {
            throw notSimulated("resetDeviceConfigurationForOpMode");
        }

        @Override
        public void setMotorVelocity(int motor, double ticksPerSecond) {
            throw notSimulated("setMotorVelocity");
        }

        @Override
        public void setMotorVelocity(int motor, double angularRate, AngleUnit unit) {
            throw notSimulated("setMotorVelocity");
        }

        @Override
        public double getMotorVelocity(int motor) {
            throw notSimulated("getMotorVelocity");
        }

        @Override
        public double getMotorVelocity(int motor, AngleUnit unit) {
            throw notSimulated("getMotorVelocity");
        }

        @Override
        public void setPIDCoefficients(int motor, DcMotor.RunMode mode, PIDCoefficients pidCoefficients) {
            throw notSimulated("setPIDCoefficients");
        }

        @Override
        public void setPIDFCoefficients(int motor, DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) {
            throw notSimulated("setPIDFCoefficients");
        }

        @Override
        public PIDCoefficients getPIDCoefficients(int motor, DcMotor.RunMode mode) {
            throw notSimulated("getPIDCoefficients");
        }

        @Override
        public PIDFCoefficients getPIDFCoefficients(int motor, DcMotor.RunMode mode) {
            throw notSimulated("getPIDFCoefficients");
        }

        @Override
        public void setMotorTargetPosition(int motor, int position, int tolerance) {
            throw notSimulated("setMotorTargetPosition");
        }

        @Override
        public double getMotorCurrentAlert(int motor, CurrentUnit unit) {
            throw notSimulated("getMotorCurrentAlert");
        }

        @Override
        public void setMotorCurrentAlert(int motor, double current, CurrentUnit unit) {
            throw notSimulated("setMotorCurrentAlert");
        }

        @Override
        public boolean isMotorOverCurrent(int motor) {
            throw notSimulated("isMotorOverCurrent");
        }
    }
}
