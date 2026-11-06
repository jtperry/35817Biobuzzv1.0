package org.biobuzz.sim;

/**
 * The flywheel as a first-order plant: its speed chases the steady speed the applied volts hold, with one time
 * constant. Every number is a PLACEHOLDER until robot logs exist.
 */
public final class FlywheelModel {

    /** PLACEHOLDER: volts lost to friction before the wheel turns. Flywheel.KS_VOLTS, the code's own fit. */
    public static final double KS_VOLTS = 1.4145;
    /** PLACEHOLDER: volts per RPM of steady speed. Flywheel.KV_VOLTS_PER_RPM, the code's own fit. */
    public static final double KV_VOLTS_PER_RPM = 0.0020984;
    /** PLACEHOLDER (invented): spin-up time constant, s; one of them reaches 63% of a step. */
    public static final double TIME_CONSTANT_S = 0.5;

    private double rpm = 0;

    /**
     * Steady RPM that power x volts holds: (|power| x volts - KS) / KV, signed like power; 0 inside friction. The
     * same form as the code's feedforward (KS + KV x RPM), so its speed loop can settle on target.
     */
    public static double steadyRpm(double power, double volts) {
        double drive = Math.abs(power) * volts - KS_VOLTS;
        return drive <= 0 ? 0 : Math.copySign(drive / KV_VOLTS_PER_RPM, power);
    }

    /** Advances dt seconds at this power: the exact first-order step. A tick that is not forward in time changes nothing. */
    public void step(double power, double volts, double dt) {
        if (!(dt > 0) || Double.isInfinite(dt)) {
            return;
        }
        rpm += (steadyRpm(power, volts) - rpm) * (1 - Math.exp(-dt / TIME_CONSTANT_S));
    }

    /** Wheel speed, RPM, positive = the shooting direction. */
    public double rpm() {
        return rpm;
    }
}
