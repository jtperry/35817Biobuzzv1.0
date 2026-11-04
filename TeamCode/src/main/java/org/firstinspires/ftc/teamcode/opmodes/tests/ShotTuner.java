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
import org.firstinspires.ftc.teamcode.tuning.ButtonArming;
import org.firstinspires.ftc.teamcode.tuning.ShotTunerLane;
import org.firstinspires.ftc.teamcode.tuning.ShotTunerLog;
import org.firstinspires.ftc.teamcode.tuning.ShotTunerRoute;
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
 * INIT: gamepad 1 dpad left/right = BLUE/RED (locked while an auto pose waits for Main TeleOp), gamepad 2 B / X / dpad
 * = start slot (as in Main TeleOp; slot 0 unless picked).
 * After PLAY, gamepad 1 (a button held through PLAY does nothing until it is released):
 *   A = first press: drive to the first unsaved spot; after that: save RPM + hood, drive to the next spot
 *   X = skip (no save) and drive on    Y = back one spot (a re-save overwrites)    B = drive back to the spot
 *   dpad up/down = RPM +-RPM_STEP      dpad right/left = hood +-HOOD_STEP
 *   RT hold = shoot (teleop gating, at least MIN_SHOT_RPM, not while moving)    LB = intake    RB = eject
 *   sticks = stop the move, field-centric manual drive until A / X / Y / B
 * Moves follow ShotTunerRoute's plan; a blocked plan drives nothing. Always aims at cell A. Saves are appended to
 * /sdcard/FIRST/shot-tuner.csv; the next INIT resumes at the first unsaved spot.
 */
@Configurable
@NextTeleop(name = "Shot Tuner", group = "test")
public class ShotTuner extends NextOpMode {

    /**
     * Path speed cap, percent, used as 5-100 (not finite: 40); also caps the final hold until the robot has settled at
     * the spot. Once settled, the hold at the spot runs at full power (recoil).
     */
    public static double DRIVE_PCT = 40;
    /** Applied only when the result is finite. */
    public static double RPM_STEP = 50;
    /** Applied only when the result is finite. */
    public static double HOOD_STEP = 0.005;
    /** Telemetry warns below this. Only after PLAY: the battery is not read in INIT. */
    public static double MIN_BATTERY_V = 12.0;
    /** A stick past this stops the move and gives manual drive. Used as 0.05-0.9 (not finite: 0.2). */
    public static double STICK_CANCEL = 0.2;
    /** RT shoots only at or above this target rpm (well below the lowest ShotTable RPM). Below 500 or not finite: 1000. */
    public static double MIN_SHOT_RPM = 1000;

    private static final File FILE = new File(AppUtil.FIRST_FOLDER, "shot-tuner.csv");
    private static final double DEFAULT_DRIVE_LIMIT = 0.40;
    private static final double DEFAULT_STICK_CANCEL = 0.2;
    private static final double DEFAULT_MIN_SHOT_RPM = 1000;
    private static final double LOWEST_MIN_SHOT_RPM = 500;
    /** A move that goes less than this far and turns less than STALL_TURN_RAD for STALL_NANOS is stopped. */
    private static final double STALL_MOVE_IN = 1.0;
    private static final double STALL_TURN_RAD = Math.toRadians(3);
    private static final long STALL_NANOS = 3_000_000_000L;
    /** A move ends (arrive()) once the robot is this close to the spot, or SETTLE_NANOS after the final hold began. */
    private static final double ARRIVED_IN = 1.0;
    private static final double ARRIVED_RAD = Math.toRadians(2);
    private static final long SETTLE_NANOS = 2_000_000_000L;
    /** After the session is done, "coasting down" while the measured rpm is above this. */
    private static final double STOPPED_RPM = 100;
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
    /** The latest row per nominal (refreshTables). */
    private Map<Double, ShotTunerLog.Row> latest;

    private List<ShotTunerLane.Spot> spots;
    private ShotTunerSession session;
    /** Cell A and the turn point, frozen with the spots (live in INIT frames, frozen at PLAY by start()). */
    private Pose laneCell;
    private Pose laneTurn;
    private String rpmTable = "";
    private String hoodTable = "";
    private String saveError = "";
    /** Why the last move was refused or stopped; "" when none. */
    private String moveProblem = "";

    private Command move;
    /** The spot the robot last arrived at; null while moving and after manual drive. Picks the route. */
    private ShotTunerLane.Spot heldSpot;
    /** A move was started since PLAY: from then on A saves. */
    private boolean started = false;
    private boolean manual = false;
    /** The spot and plan of the move last scheduled, for the screen. */
    private double moveNominal = Double.NaN;
    private String moveSummary = "";
    private double moveLengthIn = 0;
    /** Pose and time of the running move's last progress (no-progress watchdog). */
    private double progressX;
    private double progressY;
    private double progressH;
    private long progressNanos;
    private double rpm = 0;
    private double hood = Hood.stowPosition();
    /** The spot rpm and hood were seeded for, so B back to the same spot keeps the tuned values. */
    private double valuesFor = Double.NaN;
    /** Where rpm and hood were seeded from: "from saved row" or "from table"; "" before the first seed. */
    private String seedSource = "";

    // Every button that acts is armed only after it has been seen released after PLAY (ButtonArming). Unarmed until
    // start() reads the pads, so the bindings' state recorded at construction is false. A, X, Y and B act from the
    // loop; the others through bindings, which check armed().
    private final ButtonArming aButton = new ButtonArming(() -> gamepad1.aWasPressed(), () -> gamepad1.a);
    private final ButtonArming xButton = new ButtonArming(() -> gamepad1.xWasPressed(), () -> gamepad1.x);
    private final ButtonArming yButton = new ButtonArming(() -> gamepad1.yWasPressed(), () -> gamepad1.y);
    private final ButtonArming bButton = new ButtonArming(() -> gamepad1.bWasPressed(), () -> gamepad1.b);
    private final ButtonArming rtButton = new ButtonArming(() -> gamepad1.right_trigger > 0.5);
    private final ButtonArming lbButton = new ButtonArming(() -> gamepad1.left_bumper);
    private final ButtonArming rbButton = new ButtonArming(() -> gamepad1.right_bumper);
    private final ButtonArming upButton = new ButtonArming(() -> gamepad1.dpad_up);
    private final ButtonArming downButton = new ButtonArming(() -> gamepad1.dpad_down);
    private final ButtonArming leftButton = new ButtonArming(() -> gamepad1.dpad_left);
    private final ButtonArming rightButton = new ButtonArming(() -> gamepad1.dpad_right);
    private final ButtonArming[] bindingButtons = {
            rtButton, lbButton, rbButton, upButton, downButton, leftButton, rightButton};
    /** RT was released (<= 0.5) since the last move was scheduled. */
    private boolean rtReleasedSinceMove = true;

    /** rtShoots() was true last loop. */
    private boolean volleyHeld = false;
    /** The wheel has been at speed for the volley's target since the volley (re)started. */
    private boolean volleyLive = false;
    /** The target rpm the current volley is measured at. */
    private double volleyRpm = Double.NaN;
    private double volleyMinRpm = Double.NaN;
    /** When the running move's final hold began (its settle wait). */
    private long settleStartNanos;

    public ShotTuner(Robot robot) {
        super(robot, BulkReadHook.INSTANCE, robot.initCheck());
        this.robot = robot;
        tele = new RobotTelemetry(telemetry);
        robot.bindHardware(hardwareMap);
        // Always cell A, never pickCellBySide(): the lane and the saved distances are to cell A.
        robot.setCell(HiveCell.A);
        // Slot 0 is the documented start; the slot is otherwise remembered from earlier OpModes. The picker changes it.
        robot.setRestartSlot(0);
        robot.initFollower(hardwareMap, Field.startPose(robot.alliance()));

        rows = new ArrayList<>(log.load());
        refreshTables();
        buildSession();

        pad = new CommandGamepad(gamepad1);
        // Shoot, intake and eject as in Main TeleOp: RT feeds through the same FeedGate (see rtShoots()).
        pad.rightTrigger().isOver(0.5).and(() -> rtShoots()).whileTrue(ShootCommands.whileHeld(robot));
        pad.leftBumper().and(lbButton::armed).whileTrue(robot.intake.intake());
        pad.rightBumper().and(rbButton::armed).whileTrue(robot.intake.eject());
        pad.dpadUp().and(upButton::armed).onTrue(Commands.instant(() -> stepRpm(RPM_STEP)));
        pad.dpadDown().and(downButton::armed).onTrue(Commands.instant(() -> stepRpm(-RPM_STEP)));
        pad.dpadRight().and(rightButton::armed).onTrue(Commands.instant(() -> stepHood(HOOD_STEP)));
        pad.dpadLeft().and(leftButton::armed).onTrue(Commands.instant(() -> stepHood(-HOOD_STEP)));
    }

    @Override
    public void disabledPeriodic() {
        // Triggers do not poll in init: read the pads directly. The alliance only when no auto pose is pending (as in
        // Main TeleOp): that pose and its alliance wait for Main TeleOp.
        if (!robot.hasAutoHandoff()) {
            if (gamepad1.dpad_left) {
                robot.setAlliance(Alliance.BLUE);
            } else if (gamepad1.dpad_right) {
                robot.setAlliance(Alliance.RED);
            }
        }
        // The slot stays live: the first move always starts from the picked slot, never from the auto pose.
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
        tele.line("Shot Tuner DRIVES after PLAY when A, X, Y or B is pressed, straight to the lane.");
        tele.line("Touch a stick = stop the move.");
        tele.line("The first move is planned from the picked start slot: the robot must stand exactly there. Start at "
                + "slot 0 (right) after a battery swap. A move whose path would hit the hive or a wall is refused.");
        Pose startPose = Field.restartPose(robot.alliance(), robot.restartSlot());
        ShotTunerLane.Spot first = session.current();
        if (first != null) {
            ShotTunerRoute.Plan plan = ShotTunerRoute.plan(startPose, null, first, laneCell, laneTurn, true);
            if (plan.blocked) {
                tele.line("WARNING: from this start slot the first move is refused (" + plan.reason + "): after PLAY "
                        + "drive to the lane by hand, then press B.");
            }
        }
        tele.line("Keep ShotTable.RPM_SCALE = 1.0 while tuning: saved RPM includes the scale.");
        tele.line(String.format(Locale.US, "RT shoots only while the target rpm is at least MIN_SHOT_RPM (%.0f; it is "
                + "0 until the first spot is reached).", minShotRpm()));
        tele.line("RT force-feeds after 2 s even if the wheel is not at speed: check the flywheel encoder sign and run "
                + "the Flywheel Tuner first.");
        if (robot.hasAutoHandoff()) {
            tele.line("Alliance locked: an auto ran and its end pose is waiting for Main TeleOp.");
        }
        tele.add("alliance", robot.alliance() + (robot.hasAutoHandoff() ? "   (locked)"
                : "   (dpad left = BLUE, right = RED)"));
        tele.add("start slot", robot.restartSlot() + " " + Field.RESTART_SLOT_NAMES[robot.restartSlot()]
                + "   (gamepad 2: B / X / dpad)");
        addProgress();
        addTables();
        drawField(startPose);
        tele.endFrame();
    }

    @Override
    public void start() {
        // No drive command here (a STOP during INIT also runs start()): the first move waits for a button.
        buildSession();
        // Pose = the picked slot, held where the robot stands (as in Main TeleOp).
        robot.resetPoseToStart();
        robot.reset();
        robot.flywheel.fixed(() -> rpm);
        setHood(Hood.stowPosition());
        // A button held through PLAY must not act until it is released and pressed again. Draining the edge flags here
        // does not work, for two reasons:
        // 1. SDK 12 resets gamepad edge detection in OpModeInternal.internalStart(), before this start() runs. The
        //    Driver Station resends a pad every 40 ms unless it has been unchanged for over 1 s with the sticks and
        //    triggers at zero (buttons do not count), and each resend latches a fresh press of a held button.
        // 2. NextFTC bindings record their state when they are bound in the constructor and poll first after PLAY, so
        //    a button pressed during INIT and still held reads as a rising edge.
        // So every button that acts is armed only after it has been seen released after PLAY (ButtonArming, polled
        // in periodic()).
        aButton.start();
        xButton.start();
        yButton.start();
        bButton.start();
        for (ButtonArming button : bindingButtons) {
            button.start();
        }
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
        checkProgress();
        // Edge reads here, not bindings: a move is scheduled from the loop, never from inside a running command.
        // ButtonArming reads (and so consumes) each edge every loop, and acts only when its button was armed at the
        // start of this loop.
        if (aButton.poll()) {
            saveOrGo();
        }
        if (xButton.poll()) {
            if (session.current() != null) {
                session.skip();
                stopFlywheelIfDone();
            }
            driveToCurrent();
        }
        if (yButton.poll()) {
            session.back();
            driveToCurrent();
        }
        if (bButton.poll()) {
            driveToCurrent();
        }
        // The bindings' arming (their triggers poll after periodic()).
        for (ButtonArming button : bindingButtons) {
            button.poll();
        }
        if (gamepad1.right_trigger <= 0.5) {
            rtReleasedSinceMove = true;
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
        session = new ShotTunerSession(spots, latest.keySet());
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
        stopFlywheelIfDone();
        driveToCurrent();
    }

    /** After a save or a skip: a done session stops the flywheel; Y back then seeds the spot again. */
    private void stopFlywheelIfDone() {
        if (session.current() == null) {
            setRpm(0);
            valuesFor = Double.NaN;
        }
    }

    private void driveToCurrent() {
        ShotTunerLane.Spot spot = session.current();
        if (spot == null) {
            moveProblem = "";
            // X on the last spot while driving to it: the session is done, so stop the old move.
            if (moving()) {
                stopMove();
                heldSpot = null;
                robot.follower().hold(robot.pose());
            }
            return;
        }
        ShotTunerRoute.Plan plan = ShotTunerRoute.plan(robot.pose(), heldSpot, spot, laneCell, laneTurn, !started);
        boolean wasMoving = moving();
        stopMove();
        if (plan.blocked) {
            // Drive nothing: the driver gets closer to the lane by hand.
            moveProblem = "PATH BLOCKED: " + plan.reason + ". Drive closer to the lane by hand, then press B.";
            if (wasMoving) {
                heldSpot = null;
                robot.follower().hold(robot.pose());
            }
            return;
        }
        moveProblem = "";
        move = moveTo(spot, plan);
        moveNominal = spot.nominalIn;
        moveSummary = plan.summary;
        moveLengthIn = plan.lengthIn;
        heldSpot = null;
        manual = false;
        started = true;
        rtReleasedSinceMove = false;
        volleyMinRpm = Double.NaN;
        Pose now = robot.pose();
        markProgress(now.x(), now.y(), now.heading());
        move.schedule();
    }

    /**
     * The plan as commands: the drive limit, a path per translating leg (an in-place leg is left to the final hold),
     * the hold at the spot, a wait until the robot has settled there (still under the drive limit, and still moving()
     * for the stick stop and the watchdog), then arrive().
     */
    private Command moveTo(ShotTunerLane.Spot spot, ShotTunerRoute.Plan plan) {
        Follower follower = robot.follower();
        Pose cell = laneCell;
        // Same as AutoCommands.limitPower (private there); the limit is read when the command runs.
        Command drive = Commands.instant(() -> LimitedMecanum.maxPower = driveLimit());
        for (ShotTunerRoute.Leg leg : plan.legs) {
            if (leg.inPlace()) {
                continue;
            }
            if (leg.facingCell) {
                drive = drive.then(follow(follower, Paths.line(leg.from, leg.to).facingPoint(cell)));
            } else {
                drive = drive.then(follow(follower, Paths.line(leg.from, leg.to).linear(leg.from, leg.to)));
            }
        }
        Pose goal = ShotTunerLane.pose(spot);
        // Ivy's hold(follower, pose) is instant: without the wait, arrive() would lift the drive limit while the hold
        // still turns or settles. The wait restarts the watchdog's progress mark, so its SETTLE_NANOS bound always
        // ends it before STALL_NANOS can (a capped hold may sit still just outside ARRIVED_IN / ARRIVED_RAD).
        return drive.then(hold(follower, goal))
                .then(Commands.waitUntil(() -> settled(goal)).setStart(() -> {
                    settleStartNanos = System.nanoTime();
                    Pose now = robot.pose();
                    markProgress(now.x(), now.y(), now.heading());
                }))
                .then(Commands.instant(() -> arrive(spot)));
    }

    /** At the goal, or SETTLE_NANOS after the final hold began. */
    private boolean settled(Pose goal) {
        return atSpot(goal) || System.nanoTime() - settleStartNanos >= SETTLE_NANOS;
    }

    /** Within ARRIVED_IN and ARRIVED_RAD of the spot's pose. */
    private boolean atSpot(Pose spot) {
        Pose now = robot.pose();
        return Math.hypot(now.x() - spot.x(), now.y() - spot.y()) <= ARRIVED_IN
                && Math.abs(MathUtil.wrapRadians(now.heading() - spot.heading())) <= ARRIVED_RAD;
    }

    /** No-progress watchdog: a running move that neither moves nor turns for STALL_NANOS is stopped. */
    private void checkProgress() {
        if (!moving()) {
            return;
        }
        Pose now = robot.pose();
        if (Math.hypot(now.x() - progressX, now.y() - progressY) >= STALL_MOVE_IN
                || Math.abs(MathUtil.wrapRadians(now.heading() - progressH)) >= STALL_TURN_RAD) {
            markProgress(now.x(), now.y(), now.heading());
        } else if (System.nanoTime() - progressNanos >= STALL_NANOS) {
            stopMove();
            heldSpot = null;
            robot.follower().hold(now);
            moveProblem = "STOPPED: no progress for 3 s (blocked?)";
        }
    }

    private void markProgress(double x, double y, double heading) {
        progressX = x;
        progressY = y;
        progressH = heading;
        progressNanos = System.nanoTime();
    }

    /** End of a move: full power for the hold (recoil), and rpm + hood seeded at a new spot. */
    private void arrive(ShotTunerLane.Spot spot) {
        LimitedMecanum.maxPower = 1.0;
        heldSpot = spot;
        if (spot.nominalIn != valuesFor) {
            // A saved spot starts from its saved values, else from the table.
            ShotTunerLog.Row saved = latest.get(spot.nominalIn);
            if (saved != null && Double.isFinite(saved.rpm) && saved.rpm >= 0 && Double.isFinite(saved.hood)) {
                setRpm(saved.rpm);
                setHood(saved.hood);
                seedSource = "from saved row";
            } else {
                double distance = robot.shot().distanceIn;
                setRpm(ShotTable.rpm(distance));
                setHood(ShotTable.hood(distance));
                seedSource = "from table";
            }
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

    /** Dpad rpm step, applied only when the result is finite; never below 0. */
    private void stepRpm(double step) {
        double next = rpm + step;
        if (Double.isFinite(next)) {
            setRpm(Math.max(0, next));
        }
    }

    /**
     * Every target rpm change goes through here: a change restarts the volley minimum, RT held or not, so a saved
     * min_rpm was always measured at the saved rpm.
     */
    private void setRpm(double target) {
        if (target != rpm) {
            volleyMinRpm = Double.NaN;
        }
        rpm = target;
    }

    /** Dpad hood step, applied only when the result is finite (setHood clamps it). */
    private void stepHood(double step) {
        double next = hood + step;
        if (Double.isFinite(next)) {
            setHood(next);
        }
    }

    /** DRIVE_PCT as a power cap: 5-100 %, 40 % when not finite. */
    private static double driveLimit() {
        double pct = DRIVE_PCT;
        return Double.isFinite(pct) ? MathUtil.clamp(pct, 5, 100) / 100 : DEFAULT_DRIVE_LIMIT;
    }

    /** STICK_CANCEL clamped to 0.05-0.9, 0.2 when not finite. */
    private static double stickCancel() {
        double cancel = STICK_CANCEL;
        return Double.isFinite(cancel) ? MathUtil.clamp(cancel, 0.05, 0.9) : DEFAULT_STICK_CANCEL;
    }

    /** MIN_SHOT_RPM when finite and at least 500, else 1000. */
    private static double minShotRpm() {
        double min = MIN_SHOT_RPM;
        return Double.isFinite(min) && min >= LOWEST_MIN_SHOT_RPM ? min : DEFAULT_MIN_SHOT_RPM;
    }

    /**
     * The RT binding's full condition, also what trackVolley counts as held: RT pressed, armed after PLAY, released
     * since the last move started, no move running, and a real shot speed.
     */
    private boolean rtShoots() {
        return gamepad1.right_trigger > 0.5 && rtButton.armed() && rtReleasedSinceMove && !moving()
                && Double.isFinite(rpm) && rpm >= minShotRpm();
    }

    private boolean sticksMoved() {
        double cancel = stickCancel();
        return Math.abs(gamepad1.left_stick_x) > cancel || Math.abs(gamepad1.left_stick_y) > cancel
                || Math.abs(gamepad1.right_stick_x) > cancel;
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

    /**
     * Lowest flywheel RPM while RT shoots: the dip of the last volley. A new volley, or a target change during one,
     * starts over, so a volley that never reaches its target records NaN (setRpm also clears it on any target change).
     */
    private void trackVolley() {
        if (!rtShoots()) {
            volleyHeld = false;
            volleyLive = false;
            return;
        }
        if (!volleyHeld || rpm != volleyRpm) {
            volleyHeld = true;
            volleyLive = false;
            volleyRpm = rpm;
            volleyMinRpm = Double.NaN;
        }
        // Start counting once the wheel is at speed, so the ramp-up is not taken for the dip. targetRpm() == rpm: a
        // dpad step runs after this loop's flywheel periodic(), so atSpeed() may still be for the old target.
        if (!volleyLive && robot.flywheel.atSpeed() && robot.flywheel.targetRpm() == rpm) {
            volleyLive = true;
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
        if (!moveProblem.isEmpty()) {
            tele.line(moveProblem);
        }
        ShotTunerLane.Spot current = session.current();
        if (current == null && rpm == 0) {
            // Measured, either sign (a reversed encoder reads negative): the wheel coasts for a few seconds.
            tele.line(Math.abs(robot.flywheel.rpm()) > STOPPED_RPM ? "Session done: flywheel coasting down."
                    : "Session done: flywheel stopped.");
        }
        tele.add("drive", driveLine(current, current != null && current.nominalIn == valuesFor));
        tele.add("drive limit", String.format(Locale.US, "%.0f %%   stick stop above %.2f",
                driveLimit() * 100, stickCancel()));
        tele.add("rpm", String.format(Locale.US, "target %.0f   now %.0f   last volley min %.0f",
                rpm, robot.flywheel.rpm(), volleyMinRpm));
        tele.add("hood", String.format(Locale.US, "%.3f", hood));
        if (current != null && current.nominalIn == valuesFor) {
            tele.add("start values", seedSource);
        }
        tele.add("distance (in)", String.format(Locale.US, "%.1f", robot.shot().distanceIn));
        tele.add("battery (V)", String.format(Locale.US, "%.2f", volts));
        tele.add("OctoQuad bad reads", robot.flywheel.badReads());
        if (!saveError.isEmpty()) {
            tele.add("NOT SAVED", saveError);
        }
        tele.line("Always aims at cell A. If the hive tips, tip it back by hand.");
        tele.line("A save + next   X skip   Y back   B return   RT shoot   LB intake   RB eject   dpad RPM / hood");
        tele.line(String.format(Locale.US, "RT shoots only while the target rpm is at least MIN_SHOT_RPM (%.0f), "
                + "not while moving; release RT after a move.", minShotRpm()));
        addTables();
        drawField(robot.pose());
        tele.endFrame();
    }

    /** What the drive is doing and what the buttons do now. tuned = rpm and hood were seeded for current. */
    private String driveLine(ShotTunerLane.Spot current, boolean tuned) {
        if (!started) {
            return "A or B = drive to the spot, X = skip it and drive on, Y = back one spot and drive";
        }
        if (manual) {
            return tuned ? "manual (A = save here, X / Y / B = back to the lane)"
                    : "manual (A / X / Y / B = back to the lane)";
        }
        if (moving()) {
            return String.format(Locale.US, "driving to %.1f in: %s, %.0f in. Touch a stick = stop the move.",
                    moveNominal, moveSummary, moveLengthIn);
        }
        if (heldSpot != null || (current != null && atSpot(ShotTunerLane.pose(current)))) {
            return "holding the spot";
        }
        if (current == null) {
            return "stopped";
        }
        // The watchdog stopped the move, or a move was refused while one ran: the robot holds where it stopped.
        return tuned ? "stopped short of the spot (A = save here, B = retry)"
                : "stopped short of the spot (A / B = retry)";
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
        tele.line("In ShotTable.RPM / ShotTable.HOOD replace only the .put(...) points inside the printed span and keep "
                + "the points outside it (today near 20.7 in and far 91.1 in): distances outside the span hold the "
                + "end values.");
        tele.add("RPM", rpmTable);
        tele.add("HOOD", hoodTable);
    }

    private void refreshTables() {
        latest = ShotTunerLog.latestByNominal(rows);
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
