package org.biobuzz.sim;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.horizon36596.simloop.fakehardware.FakeDevice;

/**
 * A NextServo's device: the SDK's own ServoImplEx (NextServo casts to it) over a fake hub, so direction, range
 * scaling and clipping are the SDK's. The hub keeps what it was sent. A servo sends no feedback and our code reads
 * none (turret, hood and flower "settled" are slew estimates in the code), so travel is not modelled: the hub
 * position is the command. Calls our code never makes throw UnsupportedOperationException with the method name.
 */
public final class SimServo extends ServoImplEx implements FakeDevice {

    public static final int PORT = 0;

    private final Hub hub;

    public SimServo() {
        this(new Hub());
    }

    private SimServo(Hub hub) {
        super(hub, PORT, new ServoConfigurationType());
        this.hub = hub;
    }

    /** The position the hub was last sent (0-1, direction applied); NaN before the first, or after a forget. */
    public double hubPosition() {
        return hub.position;
    }

    public PwmControl.PwmRange hubPwmRange() {
        return hub.pwmRange;
    }

    public boolean hubPwmEnabled() {
        return hub.pwmEnabled;
    }

    /** The SDK's version reads an Android resource. */
    @Override
    public String getDeviceName() {
        return "SimServo";
    }

    /** Nothing to advance: travel is not modelled (class javadoc). */
    @Override
    public void update(double deltaTime) {
    }

    /**
     * One port of a fake Lynx servo controller. A static class: ServoImplEx's constructor calls setServoType on it
     * before SimServo's own fields exist.
     */
    static final class Hub implements ServoControllerEx {

        private double position = Double.NaN;
        private PwmControl.PwmRange pwmRange = PwmControl.PwmRange.defaultRange;
        /** Off until the first position, as on a hub. */
        private boolean pwmEnabled = false;

        private static UnsupportedOperationException notSimulated(String method) {
            return new UnsupportedOperationException("SimServo hub: " + method + " is not simulated");
        }

        // ---- what our code reaches, and the plain reads of that state

        /** ServoImplEx's constructor sends it; the type changes nothing the sim models. */
        @Override
        public void setServoType(int servo, ServoConfigurationType servoType) {
        }

        @Override
        public void setServoPosition(int servo, double position) {
            this.position = position;
            pwmEnabled = true;
        }

        @Override
        public double getServoPosition(int servo) {
            return position;
        }

        /** ServoImpl.setDirection calls it when the direction changes. */
        @Override
        public void forgetLastKnownPosition(int servo) {
            position = Double.NaN;
        }

        @Override
        public void setServoPwmRange(int servo, PwmControl.PwmRange range) {
            pwmRange = range;
        }

        @Override
        public PwmControl.PwmRange getServoPwmRange(int servo) {
            return pwmRange;
        }

        @Override
        public void setServoPwmEnable(int servo) {
            pwmEnabled = true;
        }

        @Override
        public void setServoPwmDisable(int servo) {
            pwmEnabled = false;
        }

        @Override
        public boolean isServoPwmEnabled(int servo) {
            return pwmEnabled;
        }

        // ---- HardwareDevice

        @Override
        public HardwareDevice.Manufacturer getManufacturer() {
            return HardwareDevice.Manufacturer.Other;
        }

        @Override
        public String getDeviceName() {
            return "SimServo hub";
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
        public void pwmEnable() {
            throw notSimulated("pwmEnable");
        }

        @Override
        public void pwmDisable() {
            throw notSimulated("pwmDisable");
        }

        @Override
        public PwmStatus getPwmStatus() {
            throw notSimulated("getPwmStatus");
        }

        @Override
        public void setPulseWidth(int servo, double pulseWidth) {
            throw notSimulated("setPulseWidth");
        }

        @Override
        public double getPulseWidth(int servo) {
            throw notSimulated("getPulseWidth");
        }
    }
}
