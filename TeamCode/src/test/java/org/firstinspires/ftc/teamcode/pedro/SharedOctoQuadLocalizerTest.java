package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad.LocalizerStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Proxy;

/** The OctoQuad status wait at INIT (spec 8), against a fake OctoQuad that reports scripted statuses. */
public class SharedOctoQuadLocalizerTest {

    private static final double TIMEOUT_MS = 300;

    /** Statuses the fake reports, in order; the last one repeats. */
    private LocalizerStatus[] statuses;
    private long statusReads;

    private double savedTimeoutMs;

    @Before
    public void setUp() {
        savedTimeoutMs = SharedOctoQuadLocalizer.RUNNING_TIMEOUT_MS;
        // Short, so the timeout tests stay quick; every assertion below is relative to it.
        SharedOctoQuadLocalizer.RUNNING_TIMEOUT_MS = TIMEOUT_MS;
    }

    @After
    public void tearDown() {
        SharedOctoQuadLocalizer.RUNNING_TIMEOUT_MS = savedTimeoutMs;
    }

    private SharedOctoQuadLocalizer init() {
        OctoQuad octoQuad = (OctoQuad) Proxy.newProxyInstance(OctoQuad.class.getClassLoader(),
                new Class<?>[] {OctoQuad.class}, (proxy, method, args) -> {
                    if (method.getName().equals("getLocalizerStatus")) {
                        return statuses[(int) Math.min(statusReads++, statuses.length - 1)];
                    }
                    // Every other call the constructor makes is a void setter or read.
                    return null;
                });
        HardwareMap hardwareMap = new HardwareMap(null, null) {
            @Override
            public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
                return classOrInterface.cast(octoQuad);
            }
        };
        return new SharedOctoQuadLocalizer(hardwareMap, Constants.localizerConfig);
    }

    /** Expects INIT to stop with a message after the timeout; returns the message. */
    private String initFailure() {
        long start = System.nanoTime();
        try {
            init();
        } catch (IllegalStateException e) {
            double seconds = (System.nanoTime() - start) / 1e9;
            assertTrue("gave up after " + seconds + " s", seconds >= TIMEOUT_MS / 1000 && seconds < 2.0);
            assertTrue(e.getMessage(), e.getMessage().contains("'octoquad'"));
            return e.getMessage();
        }
        fail("INIT must stop when the localizer never reports RUNNING");
        return null;
    }

    @Test(timeout = 10000)
    public void runningAfterCalibrationFinishesInit() {
        statuses = new LocalizerStatus[] {
                LocalizerStatus.WARMING_UP_IMU, LocalizerStatus.CALIBRATING_IMU, LocalizerStatus.RUNNING};
        init();
        assertEquals(3, statusReads);
    }

    @Test(timeout = 10000)
    public void notAnsweringStopsInitWithAReadableMessage() {
        statuses = new LocalizerStatus[] {LocalizerStatus.INVALID};
        String message = initFailure();
        assertTrue(message, message.contains("INVALID"));
        assertTrue(message, message.contains("not answering"));
    }

    @Test(timeout = 10000)
    public void calibrationThatNeverEndsSaysSo() {
        statuses = new LocalizerStatus[] {LocalizerStatus.CALIBRATING_IMU};
        String message = initFailure();
        assertTrue(message, message.contains("CALIBRATING_IMU"));
        assertTrue(message, message.contains("RUNNING_TIMEOUT_MS"));
    }

    @Test(timeout = 10000)
    public void stopDuringTheWaitEndsItAtOnceAndKeepsTheInterrupt() {
        statuses = new LocalizerStatus[] {LocalizerStatus.INVALID};
        // STOP during INIT: the SDK interrupts the OpMode thread.
        Thread.currentThread().interrupt();
        try {
            long start = System.nanoTime();
            init();
            double seconds = (System.nanoTime() - start) / 1e9;
            assertTrue("interrupt flag kept", Thread.currentThread().isInterrupted());
            assertTrue("returned after " + seconds + " s", seconds < TIMEOUT_MS / 1000);
        } finally {
            // This is the timed test thread (the runner thread's @After cannot clear it): never leave the flag set.
            Thread.interrupted();
        }
    }
}
