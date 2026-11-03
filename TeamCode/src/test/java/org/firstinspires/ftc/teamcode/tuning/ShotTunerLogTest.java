package org.firstinspires.ftc.teamcode.tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public class ShotTunerLogTest {

    private static final double EPS = 1e-9;
    private static final String HEADER = "nominal_in,distance_in,rpm,hood,volts,min_rpm,alliance";

    @Rule
    public TemporaryFolder tmp = new TemporaryFolder();

    private Locale savedLocale;
    private Locale savedFormatLocale;
    private Locale savedDisplayLocale;
    private File file;

    @Before
    public void setUp() {
        savedLocale = Locale.getDefault();
        savedFormatLocale = Locale.getDefault(Locale.Category.FORMAT);
        savedDisplayLocale = Locale.getDefault(Locale.Category.DISPLAY);
        file = new File(tmp.getRoot(), "FIRST/shot-tuner.csv");
    }

    @After
    public void tearDown() {
        Locale.setDefault(savedLocale);
        Locale.setDefault(Locale.Category.FORMAT, savedFormatLocale);
        Locale.setDefault(Locale.Category.DISPLAY, savedDisplayLocale);
    }

    private static ShotTunerLog.Row row(double nominal, double distance, double rpm, double hood) {
        return new ShotTunerLog.Row(nominal, distance, rpm, hood, 12.5, rpm - 50, "BLUE");
    }

    private List<String> fileLines() throws IOException {
        return Files.readAllLines(file.toPath(), StandardCharsets.UTF_8);
    }

    @Test
    public void appendCreatesTheFolderAndTheHeaderThenTheRow() throws IOException {
        new ShotTunerLog(file).append(new ShotTunerLog.Row(30, 30.12, 2900, 0.15, 12.48, 2850, "BLUE"));
        assertEquals(Arrays.asList(HEADER, "30.0,30.12,2900.0,0.1500,12.48,2850.0,BLUE"), fileLines());
    }

    @Test
    public void appendKeepsEarlierRowsAndWritesTheHeaderOnce() throws IOException {
        new ShotTunerLog(file).append(row(30, 30.1, 2900, 0.15));
        new ShotTunerLog(file).append(row(35, 35.2, 2950, 0.2));
        new ShotTunerLog(file).append(row(30, 30.3, 2925, 0.155));
        List<String> lines = fileLines();
        assertEquals(4, lines.size());
        assertEquals(HEADER, lines.get(0));
        assertTrue(lines.get(1).startsWith("30.0,30.10,2900.0,"));
        assertTrue(lines.get(2).startsWith("35.0,35.20,2950.0,"));
        assertTrue(lines.get(3).startsWith("30.0,30.30,2925.0,"));
    }

    @Test
    public void loadReadsBackWhatWasAppended() throws IOException {
        ShotTunerLog log = new ShotTunerLog(file);
        log.append(new ShotTunerLog.Row(30, 30.12, 2900, 0.1525, 12.48, 2850, "BLUE"));
        log.append(new ShotTunerLog.Row(35, 35.5, 2950, 0.2, 12.31, 2875, "RED"));
        List<ShotTunerLog.Row> rows = log.load();
        assertEquals(2, rows.size());
        ShotTunerLog.Row first = rows.get(0);
        assertEquals(30.0, first.nominalIn, EPS);
        assertEquals(30.12, first.distanceIn, EPS);
        assertEquals(2900.0, first.rpm, EPS);
        assertEquals(0.1525, first.hood, EPS);
        assertEquals(12.48, first.volts, EPS);
        assertEquals(2850.0, first.minRpm, EPS);
        assertEquals("BLUE", first.alliance);
        assertEquals(35.0, rows.get(1).nominalIn, EPS);
        assertEquals("RED", rows.get(1).alliance);
    }

    @Test
    public void appendWritesTheHeaderWhenTheFileExistsButIsEmpty() throws IOException {
        tmp.newFolder("FIRST");
        assertTrue(file.createNewFile());
        new ShotTunerLog(file).append(row(30, 30.1, 2900, 0.15));
        assertEquals(Arrays.asList(HEADER, "30.0,30.10,2900.0,0.1500,12.50,2850.0,BLUE"), fileLines());
    }

    @Test
    public void appendAfterAPartialLastLineStartsANewLine() throws IOException {
        assertTrue(file.getParentFile().mkdirs());
        Files.write(file.toPath(),
                (HEADER + "\n30.0,30.10,2900.0,0.1500,12.50,2850.0,BLUE\n35.0,35.20,29").getBytes(StandardCharsets.UTF_8));
        ShotTunerLog log = new ShotTunerLog(file);
        log.append(row(40, 40.2, 3000, 0.25));
        List<ShotTunerLog.Row> rows = log.load();
        assertEquals(2, rows.size());
        assertEquals(30.0, rows.get(0).nominalIn, EPS);
        assertEquals(40.0, rows.get(1).nominalIn, EPS);
        assertEquals(3000.0, rows.get(1).rpm, EPS);
    }

    @Test
    public void aMissingFileLoadsEmpty() {
        assertTrue(new ShotTunerLog(file).load().isEmpty());
    }

    @Test
    public void aFolderInPlaceOfTheFileLoadsEmpty() throws IOException {
        File folder = tmp.newFolder("not-a-file");
        assertTrue(new ShotTunerLog(folder).load().isEmpty());
    }

    @Test
    public void theHeaderAndMalformedLinesAreSkipped() throws IOException {
        assertTrue(file.getParentFile().mkdirs());
        Files.write(file.toPath(), Arrays.asList(
                HEADER,
                "30.0,30.12,2900.0,0.1500,12.48,2850.0,BLUE",
                "",
                "garbage",
                "35.0,35.20,2950.0,0.2000,12.40,BLUE",
                "35.0,35.20,2950.0,0.2000,12.40,2900.0,BLUE,extra",
                "35,0,35,20,2950,0,0,2000,12,40,2900,0,BLUE",
                "35.0,abc,2950.0,0.2000,12.40,2900.0,BLUE",
                "40.0,40.25,3000.0,0.2500,12.30,2950.0,RED"), StandardCharsets.UTF_8);
        List<ShotTunerLog.Row> rows = new ShotTunerLog(file).load();
        assertEquals(2, rows.size());
        assertEquals(30.0, rows.get(0).nominalIn, EPS);
        assertEquals(40.25, rows.get(1).distanceIn, EPS);
    }

    @Test
    public void aNaNMinRpmRoundTrips() throws IOException {
        ShotTunerLog log = new ShotTunerLog(file);
        log.append(new ShotTunerLog.Row(30, 30.12, 2900, 0.15, 12.48, Double.NaN, "BLUE"));
        List<ShotTunerLog.Row> rows = log.load();
        assertEquals(1, rows.size());
        assertTrue(Double.isNaN(rows.get(0).minRpm));
        assertEquals(2900.0, rows.get(0).rpm, EPS);
    }

    @Test
    public void latestByNominalKeepsTheLaterRow() {
        Map<Double, ShotTunerLog.Row> latest = ShotTunerLog.latestByNominal(Arrays.asList(
                row(30, 30.1, 2900, 0.15), row(35, 35.2, 2950, 0.2), row(30, 30.3, 2925, 0.155)));
        assertEquals(Arrays.asList(30.0, 35.0), new ArrayList<>(latest.keySet()));
        assertEquals(2925.0, latest.get(30.0).rpm, EPS);
        assertEquals(30.3, latest.get(30.0).distanceIn, EPS);
        assertEquals(2950.0, latest.get(35.0).rpm, EPS);
    }

    @Test
    public void tableLinesAreSortedByDistanceAndPasteReady() {
        List<ShotTunerLog.Row> rows = Arrays.asList(
                row(40, 42.26, 3150, 0.3126), row(30, 30.44, 2900, 0.15), row(35, 35.0, 2950.4, 0.2));
        assertEquals(Arrays.asList(".put(30.4, 2900)", ".put(35.0, 2950)", ".put(42.3, 3150)"),
                ShotTunerLog.rpmTableLines(rows));
        assertEquals(Arrays.asList(".put(30.4, 0.150)", ".put(35.0, 0.200)", ".put(42.3, 0.313)"),
                ShotTunerLog.hoodTableLines(rows));
    }

    @Test
    public void tableLinesDoNotReorderTheCollectionPassedIn() {
        List<ShotTunerLog.Row> rows = Arrays.asList(
                row(40, 42.26, 3150, 0.3126), row(30, 30.44, 2900, 0.15), row(35, 35.0, 2950.4, 0.2));
        List<ShotTunerLog.Row> before = new ArrayList<>(rows);
        ShotTunerLog.rpmTableLines(rows);
        ShotTunerLog.hoodTableLines(rows);
        assertEquals(before, rows);
    }

    @Test
    public void tableLinesKeepADecimalPointUnderAGermanLocale() {
        Locale.setDefault(Locale.GERMANY);
        List<ShotTunerLog.Row> rows = Arrays.asList(row(35, 35.0, 2950, 0.2), row(30, 30.44, 2900, 0.15));
        assertEquals(Arrays.asList(".put(30.4, 2900)", ".put(35.0, 2950)"), ShotTunerLog.rpmTableLines(rows));
        assertEquals(Arrays.asList(".put(30.4, 0.150)", ".put(35.0, 0.200)"), ShotTunerLog.hoodTableLines(rows));
    }

    @Test
    public void aGermanDeviceWritesDecimalPointsAndReadsThemBack() throws IOException {
        Locale.setDefault(Locale.GERMANY);
        ShotTunerLog log = new ShotTunerLog(file);
        log.append(new ShotTunerLog.Row(30, 30.12, 2900, 0.1525, 12.48, 2850, "BLUE"));
        assertEquals("30.0,30.12,2900.0,0.1525,12.48,2850.0,BLUE", fileLines().get(1));
        List<ShotTunerLog.Row> rows = log.load();
        assertEquals(1, rows.size());
        assertEquals(30.12, rows.get(0).distanceIn, EPS);
        assertEquals(0.1525, rows.get(0).hood, EPS);
        assertEquals(12.48, rows.get(0).volts, EPS);
    }

    @Test
    public void nonFiniteValuesAreLeftOutOfTheTableLines() {
        List<ShotTunerLog.Row> rows = Arrays.asList(
                row(30, 30.44, 2900, 0.15), row(35, Double.NaN, 2950, 0.2), row(40, 40.0, Double.NaN, 0.25),
                row(45, 45.0, 3100, Double.POSITIVE_INFINITY));
        assertEquals(Arrays.asList(".put(30.4, 2900)", ".put(45.0, 3100)"), ShotTunerLog.rpmTableLines(rows));
        assertEquals(Arrays.asList(".put(30.4, 0.150)", ".put(40.0, 0.250)"), ShotTunerLog.hoodTableLines(rows));
    }
}
