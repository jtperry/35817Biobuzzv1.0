package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.Robot;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class OdometryDebugLogTest {

    @Rule
    public TemporaryFolder tmp = new TemporaryFolder();

    @Test
    public void writesHeaderAndRows() throws IOException {
        File folder = tmp.newFolder("FIRST");
        OdometryDebugLog log = new OdometryDebugLog(folder);

        Robot robot = new Robot();
        log.log(robot, "INIT", "test-note");
        log.log(robot, "ACTIVE", "second-note");
        log.close();

        File latest = log.getLatestFile();
        assertNotNull(latest);
        assertTrue(latest.isFile());

        List<String> lines = readLines(latest);
        assertTrue("At least header and 2 rows", lines.size() >= 3);
        assertTrue("Header starts with timestamp_ms", lines.get(0).startsWith("timestamp_ms,dt_ms,phase,status"));
        assertTrue("Row 1 contains INIT", lines.get(1).contains(",INIT,"));
        assertTrue("Row 1 contains test-note", lines.get(1).contains("test-note"));
        assertTrue("Row 2 contains ACTIVE", lines.get(2).contains(",ACTIVE,"));
        assertTrue("Row 2 contains second-note", lines.get(2).contains("second-note"));
    }

    private static List<String> readLines(File file) throws IOException {
        List<String> lines = new ArrayList<>();
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = reader.readLine()) != null) {
                lines.add(line);
            }
        }
        return lines;
    }
}
