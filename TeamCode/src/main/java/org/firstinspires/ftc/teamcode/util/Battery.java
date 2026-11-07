package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Configurable
public final class Battery {

    public static double READ_MS = 250;

    private final Every every = new Every(READ_MS);
    private VoltageSensor sensor;
    private double volts = 12.0;

    /** Call once per OpMode. */
    public void bind(HardwareMap hardwareMap) {
        sensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;
        every.reset();
    }

    /** Call once per loop; reads only every READ_MS. A bad read (0, NaN) keeps the last value. */
    public void update() {
        every.setPeriodMs(READ_MS);
        if (sensor == null || !every.ready()) {
            return;
        }
        double reading = sensor.getVoltage();
        if (reading > 1.0) {
            volts = reading;
        }
    }

    public double volts() {
        return volts;
    }
}
