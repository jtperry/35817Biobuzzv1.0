package org.firstinspires.ftc.teamcode.opmodes.tests;

import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.ShootCommands;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCell;
import org.firstinspires.ftc.teamcode.input.DriverInput;
import org.firstinspires.ftc.teamcode.pedro.LimitedMecanum;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.shooter.ShotTable;
import org.firstinspires.ftc.teamcode.tuning.ShotTunerLane;
import org.firstinspires.ftc.teamcode.tuning.ShotTunerLog;
import org.firstinspires.ftc.teamcode.tuning.ShotTunerSession;
import org.firstinspires.ftc.teamcode.util.FieldDrawing;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.RobotTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

/**
 * Auto-stepping shot tuner (spec 7.3). This one DRIVES the robot.
 * INIT: gamepad 1 dpad left/right = BLUE/RED, gamepad 2 B / X / dpad = start slot (as in Main TeleOp).
 * After PLAY, gamepad 1:
 *   A = first press: drive to the first unsaved spot; after that: save RPM + hood, drive to the next spot
 *   X = skip (no save) and drive on    Y = back one spot (a re-save overwrites)    B = drive back to the spot
 *   dpad up/down = RPM +-RPM_STEP      dpad right/left = hood +-HOOD_STEP
 *   RT hold = shoot (teleop gating)    LB = intake    RB = eject
 *   sticks = stop the move, field-centric manual drive until A / X / Y / B
 * Always aims at cell A. Saves are appended to /sdcard/FIRST/shot-tuner.csv; the next INIT resumes at the first
 * unsaved spot.
 */
@Configurable
@NextTeleop(name = "Shot Tuner", group = "test")
public class ShotTuner extends NextOpMode {

    /** Path speed cap, percent. The hold at a spot runs at full power against the recoil. */
    public static double DRIVE_PCT = 40;
    public static double RPM_STEP = 50;
    public static double HOOD_STEP = 0.005;
    /** Telemetry warns below this. Only after PLAY: the battery is not read in INIT. */
    public static double MIN_BATTERY_V = 12.0;
    /** A stick past this stops the move and gives manual drive. */
    public static double STICK_CANCEL = 0.2;

    private static final File FILE = new File(AppUtil.FIRST_FOLDER, "shot-tuner.csv");
    /** Shorter legs are skipped (with the defaults the 35 in spot is the turn point). */
    private static final double MIN_LEG_IN = 1.0;
    private static final double SPOT_RADIUS_IN = 1.5;
    private static final double CURRENT_RADIUS_IN = 3;
    private static final String LANE_COLOR = "#9E9E9E";
    private static final String SPOT_COLOR = "#FFC107";
    private static final String SAVED_COLOR = "#4CAF50";
    private static final String INVALID_COLOR = "#F44336";

    private final Robot robot;
    private final RobotTelemetry tele;
    private final CommandGamepad pad;
    private final ShotTunerLog log = new ShotTunerLog(FILE);
    /** Every row in the file plus this session's saves. */
    private final List<ShotTunerLog.Row> rows;

    private List<ShotTunerLane.Spot> spots;
    private ShotTunerSession session;
    /** Cell A and the turn point, frozen with the spots (live in INIT frames, frozen at PLAY by start()). */
    private Pose laneCell;
    private Pose laneTurn;
    private String rpmTable = "";
    private String hoodTable = "";
    private String saveError = "";
    /** The last move was refused: a leg would hit the hive frame or a wall. */
    private boolean blocked = false;

    private Command move;
    /** The spot the robot last arrived at; null while moving and after manual drive. Picks the route. */
    private ShotTunerLane.Spot heldSpot;
    /** A move was started since PLAY: from then on A saves. */
    private boolean started = false;
    private boolean manual = false;
    private double rpm = 0;
    private double hood = Hood.stowPosition();
    /** The spot rpm and hood were seeded for, so B back to the same spot keeps the tuned values. */
    private double valuesFor = Double.NaN;
    /** RT is held and the wheel has been at speed since it was pressed. */
    private boolean volleyLive = false;
    private double volleyMinRpm = Double.NaN;

    public ShotTuner(Robot robot) {
        super(robot, BulkReadHook.INSTANCE, robot.initCheck());
        this.robot = robot;
        tele = new RobotTelemetry(telemetry);
        robot.bindHardware(hardwareMap);
        // Always cell A, never pickCellBySide(): the lane and the saved distances are to cell A.
        robot.setCell(HiveCell.A);
        robot.initFollower(hardwareMap, Field.startPose(robot.alliance()));

        rows = new ArrayList<>(log.load());
        refreshTables();
        buildSession();

        pad = new CommandGamepad(gamepad1);
        // Shoot, intake and eject as in Main TeleOp: RT feeds through the same FeedGate. Only with a target rpm: at 0
        // the gate is never ready and would force the stages into the stopped wheel after its timeout.
        pad.rightTrigger().isOver(0.5).and(() -> rpm > 0).whileTrue(ShootCommands.whileHeld(robot));
        pad.leftBumper().whileTrue(robot.intake.intake());
        pad.rightBumper().whileTrue(robot.intake.eject());
        pad.dpadUp().onTrue(Commands.instant(() -> rpm += RPM_STEP));
        pad.dpadDown().onTrue(Commands.instant(() -> rpm = Math.max(0, rpm - RPM_STEP)));
        pad.dpadRight().onTrue(Commands.instant(() -> setHood(hood + HOOD_STEP)));
        pad.dpadLeft().onTrue(Commands.instant(() -> setHood(hood - HOOD_STEP)));
    }

    @Override
    public void disabledPeriodic() {
        // Triggers do not poll in init: read the pads directly.
        if (gamepad1.dpad_left) {
            robot.setAlliance(Alliance.BLUE);
        } else if (gamepad1.dpad_right) {
            robot.setAlliance(Alliance.RED);
        }
        if (gamepad2.b) {
            robot.setRestartSlot(0);
        } else if (gamepad2.dpad_right) {
            robot.setRestartSlot(1);
        } else if (gamepad2.dpad_left) {
            robot.setRestartSlot(2);
        } else if (gamepad2.dpad_up) {
            robot.setRestartSlot(3);
        } else if (gamepad2.dpad_down) {
            robot.setRestartSlot(4);
        } else if (gamepad2.x) {
            robot.setRestartSlot(5);
        }
        if (!tele.beginFrame()) {
            return;
        }
        // Rebuilt every frame so the spots follow the alliance and live lane tunable edits.
        buildSession();
        tele.line("Shot Tuner DRIVES after PLAY + A, straight to the lane.");
        tele.line("Start at slot 0 (right) after a battery swap. A move whose path would hit the hive or a wall is "
                + "refused.");
        Pose startPose = Field.restartPose(robot.alliance(), robot.restartSlot());
        if (!ShotTunerLane.linearLegClear(startPose, turnFacing(), startPose)) {
            tele.line("WARNING: from this start slot the first move is refused (the hive or a wall is in the way): "
                    + "after PLAY drive to the lane by hand, then press B.");
        }
        tele.line("Keep ShotTable.RPM_SCALE = 1.0 while tuning: saved RPM includes the scale.");
        tele.line("RT shoots only while the target rpm is above 0 (it is 0 until the first spot is reached).");
        tele.add("alliance", robot.alliance() + "   (dpad left = BLUE, right = RED)");
        tele.add("start slot", robot.restartSlot() + " " + Field.RESTART_SLOT_NAMES[robot.restartSlot()]
                + "   (gamepad 2: B / X / dpad)");
        addProgress();
        addTables();
        drawField(Field.restartPose(robot.alliance(), robot.restartSlot()));
        tele.endFrame();
    }

    @Override
    public void start() {
        // No drive command here (a STOP during INIT also runs start()): the first move waits for A.
        buildSession();
        // Pose = the picked slot, held where the robot stands (as in Main TeleOp).
        robot.resetPoseToStart();
        robot.reset();
        robot.flywheel.fixed(() -> rpm);
        setHood(Hood.stowPosition());
        // A press made during INIT must not start a move on the first loop. Drain the flags: resetEdgeDetection()
        // would make a button still held at PLAY look freshly pressed.
        gamepad1.aWasPressed();
        gamepad1.bWasPressed();
        gamepad1.xWasPressed();
        gamepad1.yWasPressed();
    }

    @Override
    public void periodic() {
        if (!manual && sticksMoved()) {
            stopMove();
            heldSpot = null;
            manual = true;
            // Stop the path now; drive() below switches to manual with the sticks.
            robot.follower().hold(robot.pose());
        }
        // Edge reads here, not bindings: a move is scheduled from the loop, never from inside a running command.
        if (gamepad1.aWasPressed()) {
            saveOrGo();
        }
        if (gamepad1.xWasPressed()) {
            session.skip();
            driveToCurrent();
        }
        if (gamepad1.yWasPressed()) {
            session.back();
            driveToCurrent();
        }
        if (gamepad1.bWasPressed()) {
            driveToCurrent();
        }
        if (manual) {
            drive();
        }
        trackVolley();
        pushTelemetry();
    }

    @Override
    public void end() {
        // Also puts LimitedMecanum.maxPower back to 1.0: statics outlive this OpMode.
        stopMove();
        robot.flywheel.reset();
    }

    private void buildSession() {
        spots = ShotTunerLane.spots(robot.alliance());
        laneCell = ShotTunerLane.cellA(robot.alliance());
        laneTurn = ShotTunerLane.turnPoint(robot.alliance());
        session = new ShotTunerSession(spots, ShotTunerLog.latestByNominal(rows).keySet());
    }

    /** First press: go to the spot. Then: save, and only if the file write worked, go to the next spot. */
    private void saveOrGo() {
        ShotTunerLane.Spot spot = session.current();
        if (spot == null || moving()) {
            return;
        }
        if (!started) {
            driveToCurrent();
            return;
        }
        // Only save values seeded and tuned for this spot (a stick during the move cancels it before arrive()).
        if (spot.nominalIn != valuesFor) {
            driveToCurrent();
            return;
        }
        ShotTunerLog.Row row = new ShotTunerLog.Row(spot.nominalIn, robot.shot().distanceIn, rpm, hood,
                robot.battery.volts(), volleyMinRpm, robot.alliance().name());
        try {
            log.append(row);
        } catch (IOException e) {
            saveError = e.toString();
            return;
        }
        saveError = "";
        rows.add(row);
        refreshTables();
        session.save();
        driveToCurrent();
    }

    private void driveToCurrent() {
        ShotTunerLane.Spot spot = session.current();
        if (spot == null) {
            // X on the last spot while driving to it: the session is done, so stop the old move.
            if (moving()) {
                stopMove();
                heldSpot = null;
                robot.follower().hold(robot.pose());
            }
            return;
        }
        boolean wasMoving = moving();
        stopMove();
        Command next = moveTo(spot);
        if (next == null) {
            // A leg would hit the hive or a wall: drive nothing, the driver gets closer to the lane by hand.
            blocked = true;
            if (wasMoving) {
                heldSpot = null;
                robot.follower().hold(robot.pose());
            }
            return;
        }
        blocked = false;
        move = next;
        heldSpot = null;
        manual = false;
        started = true;
        volleyMinRpm = Double.NaN;
        move.schedule();
    }

    /**
     * Straight along the leg the robot arrived on, else via the turn point. Legs from the turn point or a spot face
     * cell A. Null if a leg or the turn point would put the footprint into the hive frame or a wall.
     */
    private Command moveTo(ShotTunerLane.Spot spot) {
        // The frozen lane (see buildSession), the same one the spots were solved from.
        Pose cell = laneCell;
        Pose turn = laneTurn;
        Pose target = ShotTunerLane.pose(spot);
        Pose start = robot.pose();
        Pose from = start;
        // Same as AutoCommands.limitPower (private there); DRIVE_PCT is read when the command runs.
        Command drive = Commands.instant(() -> LimitedMecanum.maxPower = MathUtil.clamp(DRIVE_PCT / 100.0, 0, 1));
        if (heldSpot == null || onSideLeg(heldSpot, cell, turn) != onSideLeg(spot, cell, turn)) {
            Pose turnFacing = turnFacing();
            if (Math.hypot(turn.x() - from.x(), turn.y() - from.y()) >= MIN_LEG_IN) {
                // Turn linearly to the heading at the turn point, not facingPoint(cell): from restart slot 0
                // (flush in the corner) facing the cell from the start would swing the robot into both walls.
                if (!ShotTunerLane.linearLegClear(from, turnFacing, start)) {
                    return null;
                }
                drive = drive.then(follow(robot.follower(), Paths.line(from, turn).linear(from, turnFacing)));
            } else if (!ShotTunerLane.linearLegClear(turnFacing, turnFacing, start)) {
                // The turn point itself (a lane tunable can put it past a wall).
                return null;
            }
            from = turn;
        }
        drive = leg(drive, from, target, cell, start);
        if (drive == null) {
            return null;
        }
        Follower follower = robot.follower();
        return drive.then(hold(follower, target)).then(Commands.instant(() -> arrive(spot)));
    }

    /** Farther from cell A than the turn point (the turn point itself is on the back leg). */
    private static boolean onSideLeg(ShotTunerLane.Spot spot, Pose cell, Pose turn) {
        return spot.nominalIn > Math.hypot(turn.x() - cell.x(), turn.y() - cell.y()) + 1e-6;
    }

    /** Turn point of the frozen lane, facing cell A. */
    private Pose turnFacing() {
        return new Pose(laneTurn.x(), laneTurn.y(),
                Math.atan2(laneCell.y() - laneTurn.y(), laneCell.x() - laneTurn.x()));
    }

    /** Null if the leg would hit the hive or a wall. */
    private Command leg(Command drive, Pose from, Pose to, Pose cell, Pose start) {
        if (Math.hypot(to.x() - from.x(), to.y() - from.y()) < MIN_LEG_IN) {
            return drive;
        }
        if (!ShotTunerLane.facingLegClear(from, to, cell, start)) {
            return null;
        }
        return drive.then(follow(robot.follower(), Paths.line(from, to).facingPoint(cell)));
    }

    /** End of a move: full power for the hold (recoil), and rpm + hood from the table at a new spot. */
    private void arrive(ShotTunerLane.Spot spot) {
        LimitedMecanum.maxPower = 1.0;
        heldSpot = spot;
        if (spot.nominalIn != valuesFor) {
            double distance = robot.shot().distanceIn;
            rpm = ShotTable.rpm(distance);
            setHood(ShotTable.hood(distance));
            valuesFor = spot.nominalIn;
        }
    }

    private boolean moving() {
        return move != null && move.isScheduled();
    }

    private void stopMove() {
        if (move != null && move.isScheduled()) {
            move.cancel();
        }
        move = null;
        LimitedMecanum.maxPower = 1.0;
    }

    /** HOLD clamps to the hood limits; clamp here too so the value shown and saved is the one the servo gets. */
    private void setHood(double position) {
        hood = MathUtil.clamp(position, Hood.minPosition(), Hood.maxPosition());
        robot.hood.hold(hood);
    }

    private boolean sticksMoved() {
        return Math.abs(gamepad1.left_stick_x) > STICK_CANCEL || Math.abs(gamepad1.left_stick_y) > STICK_CANCEL
                || Math.abs(gamepad1.right_stick_x) > STICK_CANCEL;
    }

    /** Main TeleOp's field-centric drive. */
    private void drive() {
        // Red driver stands on the other side: flip both axes. Stick y is negative when pushed up.
        double flip = robot.alliance() == Alliance.RED ? -1 : 1;
        double scale = DriverInput.stickScale(gamepad1.left_stick_x, gamepad1.left_stick_y);
        DrivePowers powers = ManualDrive.fieldCentric(
                flip * gamepad1.left_stick_y * scale,
                flip * gamepad1.left_stick_x * scale,
                -DriverInput.turn(gamepad1.right_stick_x),
                robot.pose().heading());
        // Sticks released: hold the spot instead of coasting.
        ManualDrive.driveOrHold(robot.follower(), powers);
    }

    /** Lowest flywheel RPM while RT is held: the dip of the last volley. */
    private void trackVolley() {
        if (gamepad1.right_trigger <= 0.5) {
            volleyLive = false;
            return;
        }
        // Start counting once the wheel is at speed, so the ramp-up is not taken for the dip.
        if (!volleyLive && robot.flywheel.atSpeed()) {
            volleyLive = true;
            volleyMinRpm = Double.NaN;
        }
        if (volleyLive) {
            double now = robot.flywheel.rpm();
            volleyMinRpm = Double.isNaN(volleyMinRpm) ? now : Math.min(volleyMinRpm, now);
        }
    }

    private void pushTelemetry() {
        if (!tele.beginFrame()) {
            return;
        }
        double volts = robot.battery.volts();
        if (volts < MIN_BATTERY_V) {
            tele.line(String.format(Locale.US, "BATTERY LOW (%.2f V): swap it, robot back to the start corner, "
                    + "INIT again. Saves are kept.", volts));
        }
        addProgress();
        if (blocked) {
            tele.line("PATH BLOCKED by the hive or a wall: drive closer to the lane by hand, then press B.");
        }
        ShotTunerLane.Spot current = session.current();
        boolean tuned = current != null && current.nominalIn == valuesFor;
        tele.add("drive", !started ? "press A to drive to the spot"
                : manual ? (tuned ? "manual (A = save here, X / Y / B = back to the lane)"
                : "manual (A / X / Y / B = back to the lane)")
                : moving() ? "driving to the spot" : "holding the spot");
        tele.add("rpm", String.format(Locale.US, "target %.0f   now %.0f   last volley min %.0f",
                rpm, robot.flywheel.rpm(), volleyMinRpm));
        tele.add("hood", String.format(Locale.US, "%.3f", hood));
        tele.add("distance (in)", String.format(Locale.US, "%.1f", robot.shot().distanceIn));
        tele.add("battery (V)", String.format(Locale.US, "%.2f", volts));
        if (!saveError.isEmpty()) {
            tele.add("NOT SAVED", saveError);
        }
        tele.line("Always aims at cell A. If the hive tips, tip it back by hand.");
        tele.line("A save + next   X skip   Y back   B return   RT shoot   LB intake   RB eject   dpad RPM / hood");
        tele.line("RT shoots only while the target rpm is above 0 (it is 0 until the first spot is reached).");
        addTables();
        drawField(robot.pose());
        tele.endFrame();
    }

    private void addProgress() {
        ShotTunerLane.Spot spot = session.current();
        tele.add("spot", spot == null ? "done: nothing unsaved ahead (Y = back one spot and re-save; "
                + "delete /sdcard/FIRST/shot-tuner.csv to start over)"
                : String.format(Locale.US, "%.1f in   (%d of %d)", spot.nominalIn, session.index() + 1, spots.size()));
        tele.add("saved / valid", session.savedCount() + " / " + session.validCount());
        StringBuilder invalid = new StringBuilder();
        for (ShotTunerLane.Spot s : spots) {
            if (!s.valid) {
                invalid.append(String.format(Locale.US, "%.1f %s   ", s.nominalIn, s.reason));
            }
        }
        if (invalid.length() > 0) {
            tele.add("invalid, skipped", invalid.toString().trim());
        }
    }

    private void addTables() {
        if (rows.isEmpty()) {
            return;
        }
        tele.line("Replace the .put(...) chain after \"new LinearTable()\" in ShotTable.RPM / ShotTable.HOOD:");
        tele.add("RPM", rpmTable);
        tele.add("HOOD", hoodTable);
    }

    private void refreshTables() {
        Map<Double, ShotTunerLog.Row> latest = ShotTunerLog.latestByNominal(rows);
        rpmTable = chain(ShotTunerLog.rpmTableLines(latest.values()));
        hoodTable = chain(ShotTunerLog.hoodTableLines(latest.values()));
    }

    /** The .put(...) lines joined into one chain (no String.join: Android API 26). */
    private static String chain(List<String> lines) {
        StringBuilder text = new StringBuilder();
        for (String line : lines) {
            text.append(line);
        }
        return text.toString();
    }

    /** Lane (cell A, turn point, last spot) and spots, then the robot, which sends the frame. */
    private void drawField(Pose robotPose) {
        Pose cell = laneCell;
        Pose turn = laneTurn;
        FieldDrawing.line(cell.x(), cell.y(), turn.x(), turn.y(), LANE_COLOR);
        if (!spots.isEmpty()) {
            ShotTunerLane.Spot last = spots.get(spots.size() - 1);
            FieldDrawing.line(turn.x(), turn.y(), last.x, last.y, LANE_COLOR);
        }
        ShotTunerLane.Spot current = session.current();
        for (ShotTunerLane.Spot spot : spots) {
            String color = !spot.valid ? INVALID_COLOR : session.isSaved(spot.nominalIn) ? SAVED_COLOR : SPOT_COLOR;
            FieldDrawing.circle(spot.x, spot.y, spot == current ? CURRENT_RADIUS_IN : SPOT_RADIUS_IN, color);
        }
        FieldDrawing.robot(robotPose);
    }
}
