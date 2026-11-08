package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad.EncoderDataBlock;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad.LocalizerDataBlock;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad.LocalizerStatus;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.SharedOctoQuadLocalizer;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Logs odometry computer (OctoQuad) health, I2C read stats, status, pose, encoder ticks, and robot state
 * to a CSV file in AppUtil.FIRST_FOLDER (/sdcard/FIRST/odometry-latest.csv and a timestamped run file).
 *
 * Designed to diagnose communication drops, CRC errors, frame errors, and IMU status changes.
 */
public final class OdometryDebugLog {

    private static final String HEADER =
            "timestamp_ms,dt_ms,phase,status,loc_valid,enc_valid,total_reads,bad_loc,bad_enc,consec_bad,max_consec,"
            + "x_in,y_in,heading_deg,vx_in_s,vy_in_s,omega_deg_s,"
            + "x_pod_ticks,y_pod_ticks,flywheel_ticks,flywheel_rpm,target_rpm,battery_volts,note";

    private final File latestFile;
    private final File runFile;
    private PrintWriter latestWriter;
    private PrintWriter runWriter;
    private long startNanos = 0;
    private long lastLogNanos = 0;
    private int linesSinceFlush = 0;
    private boolean active = false;

    public OdometryDebugLog() {
        this(resolveFolder());
    }

    public OdometryDebugLog(File folder) {
        if (folder != null && !folder.exists()) {
            folder.mkdirs();
        }
        String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        this.latestFile = folder != null ? new File(folder, "odometry-latest.csv") : new File("odometry-latest.csv");
        this.runFile = folder != null ? new File(folder, "odometry-run-" + timestamp + ".csv") : null;
        initWriters();
    }

    public static File resolveFolder() {
        try {
            if (AppUtil.FIRST_FOLDER != null) {
                return AppUtil.FIRST_FOLDER;
            }
        } catch (Throwable ignored) {
        }
        return new File("build/logs");
    }

    private void initWriters() {
        try {
            latestWriter = new PrintWriter(new BufferedWriter(new FileWriter(latestFile, false)));
            latestWriter.println(HEADER);
            if (runFile != null) {
                runWriter = new PrintWriter(new BufferedWriter(new FileWriter(runFile, false)));
                runWriter.println(HEADER);
            }
            active = true;
        } catch (IOException e) {
            active = false;
        }
    }

    public synchronized void log(Robot robot, String phase) {
        log(robot, phase, "");
    }

    public synchronized void log(Robot robot, String phase, String note) {
        if (!active || robot == null) {
            return;
        }
        long now = System.nanoTime();
        if (startNanos == 0) {
            startNanos = now;
        }
        double elapsedMs = (now - startNanos) / 1e6;
        double dtMs = lastLogNanos == 0 ? 0 : (now - lastLogNanos) / 1e6;
        lastLogNanos = now;

        SharedOctoQuadLocalizer localizer = robot.localizer();
        LocalizerStatus status = LocalizerStatus.INVALID;
        boolean locValid = false;
        boolean encValid = false;
        long totalReads = 0;
        long badLoc = 0;
        long badEnc = 0;
        int consecBad = 0;
        int maxConsec = 0;
        int xTicks = 0;
        int yTicks = 0;
        int flywheelTicks = 0;

        if (localizer != null) {
            status = localizer.lastStatus();
            totalReads = localizer.totalReads();
            badLoc = localizer.badLocalizerReads();
            badEnc = localizer.badEncoderReads();
            consecBad = localizer.consecutiveBadReads();
            maxConsec = localizer.maxConsecutiveBadReads();

            LocalizerDataBlock locData = localizer.localizerData();
            EncoderDataBlock encData = localizer.encoderData();
            if (locData != null) {
                locValid = locData.isDataValid();
            }
            if (encData != null) {
                encValid = encData.crcOk;
                if (encData.positions != null && encData.positions.length >= 3) {
                    yTicks = encData.positions[0];
                    xTicks = encData.positions[1];
                    flywheelTicks = encData.positions[2];
                }
            }
        }

        double x = 0, y = 0, headingDeg = 0;
        double vx = 0, vy = 0, omegaDeg = 0;
        if (robot.follower() != null) {
            com.pedropathing.math.Pose pose = robot.follower().pose();
            if (pose != null) {
                x = pose.x();
                y = pose.y();
                headingDeg = Math.toDegrees(pose.heading());
            }
            com.pedropathing.math.Velocity vel = robot.follower().velocity();
            if (vel != null) {
                vx = vel.vx;
                vy = vel.vy;
                omegaDeg = Math.toDegrees(vel.omega);
            }
        }

        double flywheelRpm = robot.flywheel != null ? robot.flywheel.rpm() : 0;
        double targetRpm = robot.flywheel != null ? robot.flywheel.targetRpm() : 0;
        double batteryVolts = robot.battery != null ? robot.battery.volts() : 0;

        String line = String.format(Locale.US,
                "%.1f,%.2f,%s,%s,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.1f,%.2f,%.2f,%.1f,%d,%d,%d,%.1f,%.1f,%.2f,%s",
                elapsedMs, dtMs, phase, status != null ? status.name() : "NULL",
                locValid ? 1 : 0, encValid ? 1 : 0,
                totalReads, badLoc, badEnc, consecBad, maxConsec,
                x, y, headingDeg, vx, vy, omegaDeg,
                xTicks, yTicks, flywheelTicks,
                flywheelRpm, targetRpm, batteryVolts,
                note != null ? note : ""
        );

        if (latestWriter != null) {
            latestWriter.println(line);
        }
        if (runWriter != null) {
            runWriter.println(line);
        }

        linesSinceFlush++;
        if (linesSinceFlush >= 20) {
            flush();
        }
    }

    public synchronized void flush() {
        if (latestWriter != null) {
            latestWriter.flush();
        }
        if (runWriter != null) {
            runWriter.flush();
        }
        linesSinceFlush = 0;
    }

    public synchronized void close() {
        flush();
        if (latestWriter != null) {
            latestWriter.close();
            latestWriter = null;
        }
        if (runWriter != null) {
            runWriter.close();
            runWriter = null;
        }
        active = false;
    }

    public File getLatestFile() {
        return latestFile;
    }

    public File getRunFile() {
        return runFile;
    }
}
