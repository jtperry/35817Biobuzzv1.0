package org.firstinspires.ftc.teamcode.tuning;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;

/**
 * Shot Tuner saves, one CSV row per save (spec 7.3). The file is only ever appended to, so a battery swap keeps
 * every earlier save; the latest row for a spot wins. Numbers are written with Locale.US so the file reads back
 * the same on a device set to a comma-decimal language.
 */
public final class ShotTunerLog {

    private static final String HEADER = "nominal_in,distance_in,rpm,hood,volts,min_rpm,alliance";
    private static final int FIELDS = 7;

    /** One save. distanceIn is robot.shot().distanceIn, the ShotTable lookup key. */
    public static final class Row {
        public final double nominalIn;
        public final double distanceIn;
        public final double rpm;
        public final double hood;
        public final double volts;
        public final double minRpm;
        public final String alliance;

        public Row(double nominalIn, double distanceIn, double rpm, double hood,
                   double volts, double minRpm, String alliance) {
            this.nominalIn = nominalIn;
            this.distanceIn = distanceIn;
            this.rpm = rpm;
            this.hood = hood;
            this.volts = volts;
            this.minRpm = minRpm;
            this.alliance = alliance;
        }
    }

    private final File file;

    public ShotTunerLog(File file) {
        this.file = file;
    }

    /** Every row in file order. A missing or unreadable file gives an empty list; the header and bad lines are skipped. */
    public List<Row> load() {
        List<Row> rows = new ArrayList<>();
        if (!file.isFile()) {
            return rows;
        }
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = reader.readLine()) != null) {
                Row row = parse(line);
                if (row != null) {
                    rows.add(row);
                }
            }
        } catch (IOException e) {
            return new ArrayList<>();
        }
        return rows;
    }

    /**
     * Adds one row at the end. A new file gets its parent folders and the header first; a file whose last line has
     * no newline (a power cut mid-write) gets one before the row. Never truncates.
     */
    public void append(Row row) throws IOException {
        File parent = file.getParentFile();
        if (parent != null && !parent.isDirectory() && !parent.mkdirs()) {
            throw new IOException("Could not create " + parent);
        }
        boolean isNew = file.length() == 0;
        boolean needsNewline = !isNew && !endsWithNewline();
        try (Writer writer = new FileWriter(file, true)) {
            if (isNew) {
                writer.write(HEADER + "\n");
            } else if (needsNewline) {
                writer.write("\n");
            }
            writer.write(String.format(Locale.US, "%.1f,%.2f,%.1f,%.4f,%.2f,%.1f,%s\n",
                    row.nominalIn, row.distanceIn, row.rpm, row.hood, row.volts, row.minRpm, row.alliance));
        }
    }

    private boolean endsWithNewline() throws IOException {
        try (RandomAccessFile reader = new RandomAccessFile(file, "r")) {
            reader.seek(reader.length() - 1);
            return reader.read() == '\n';
        }
    }

    /** One row per nominal distance, sorted by nominal; a later row replaces an earlier one (a re-save overwrites). */
    public static Map<Double, Row> latestByNominal(List<Row> rows) {
        Map<Double, Row> latest = new TreeMap<>();
        for (Row row : rows) {
            latest.put(row.nominalIn, row);
        }
        return latest;
    }

    /** Paste-ready .put(distance, rpm) lines for the ShotTable.RPM initializer, sorted by distance. */
    public static List<String> rpmTableLines(Collection<Row> rows) {
        List<String> lines = new ArrayList<>();
        for (Row row : sortedByDistance(rows)) {
            if (Double.isFinite(row.distanceIn) && Double.isFinite(row.rpm)) {
                lines.add(String.format(Locale.US, ".put(%.1f, %.0f)", row.distanceIn, row.rpm));
            }
        }
        return lines;
    }

    /** Paste-ready .put(distance, hood) lines for the ShotTable.HOOD initializer, sorted by distance. */
    public static List<String> hoodTableLines(Collection<Row> rows) {
        List<String> lines = new ArrayList<>();
        for (Row row : sortedByDistance(rows)) {
            if (Double.isFinite(row.distanceIn) && Double.isFinite(row.hood)) {
                lines.add(String.format(Locale.US, ".put(%.1f, %.3f)", row.distanceIn, row.hood));
            }
        }
        return lines;
    }

    private static List<Row> sortedByDistance(Collection<Row> rows) {
        List<Row> sorted = new ArrayList<>(rows);
        Collections.sort(sorted, (a, b) -> Double.compare(a.distanceIn, b.distanceIn));
        return sorted;
    }

    /** null for the header, a blank line or anything that is not 7 fields of 6 numbers and an alliance. */
    private static Row parse(String line) {
        String[] parts = line.trim().split(",", -1);
        if (parts.length != FIELDS) {
            return null;
        }
        try {
            return new Row(Double.parseDouble(parts[0]), Double.parseDouble(parts[1]), Double.parseDouble(parts[2]),
                    Double.parseDouble(parts[3]), Double.parseDouble(parts[4]), Double.parseDouble(parts[5]),
                    parts[6].trim());
        } catch (NumberFormatException e) {
            return null;
        }
    }
}
