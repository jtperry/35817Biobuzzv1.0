package org.firstinspires.ftc.teamcode.autos;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

/**
 * A routine step tells the sim's sink (OpMode.blackboard) where it starts and ends, and is otherwise its body: same
 * loops, same requirements. Driven through Ivy's Scheduler by hand; "@n" is the Scheduler.execute() pass (0 = the
 * schedule call).
 */
public class StepMarksTest {

    private final List<String> log = new ArrayList<>();
    private int pass;
    private boolean hadSink;
    private Object savedSink;

    @Before
    public void setUp() {
        hadSink = OpMode.blackboard.containsKey(StepMarks.SINK_KEY);
        savedSink = OpMode.blackboard.remove(StepMarks.SINK_KEY);
        Scheduler.reset();
    }

    @After
    public void tearDown() {
        Scheduler.reset();
        if (hadSink) {
            OpMode.blackboard.put(StepMarks.SINK_KEY, savedSink);
        } else {
            OpMode.blackboard.remove(StepMarks.SINK_KEY);
        }
    }

    /** The sink the sim would put there: logs "mark id event @pass". */
    private void installSink() {
        BiConsumer<String, String> sink = (id, event) -> log.add("mark " + id + " " + event + " @" + pass);
        OpMode.blackboard.put(StepMarks.SINK_KEY, sink);
    }

    /** Logs its start and end; done after it has executed loops times (0 = done as soon as it is checked). */
    private CommandBuilder body(String name, int loops) {
        int[] executed = {0};
        return Command.build()
                .setStart(() -> log.add(name + " start @" + pass))
                .setExecute(() -> executed[0]++)
                .setDone(() -> executed[0] >= loops)
                .setEnd(condition -> log.add(name + " end " + condition + " @" + pass));
    }

    private void schedule(Command command) {
        pass = 0;
        command.schedule();
    }

    private void execute(int passes) {
        for (int i = 0; i < passes; i++) {
            pass++;
            Scheduler.execute();
        }
    }

    private void runToEnd(Command command) {
        schedule(command);
        while (command.isScheduled()) {
            assertTrue("still running after 100 passes", pass < 100);
            execute(1);
        }
    }

    private List<String> entries(boolean marks) {
        List<String> picked = new ArrayList<>();
        for (String entry : log) {
            if (entry.startsWith("mark ") == marks) {
                picked.add(entry);
            }
        }
        return picked;
    }

    @Test
    public void aStepMarksItsStartAsTheBodyStartsAndItsEndAsTheBodyEnds() {
        installSink();
        runToEnd(StepMarks.wrap("drive", body("body", 2)));
        assertEquals(Arrays.asList(
                "mark drive start @0", "body start @0", "body end NATURALLY @2", "mark drive end @2"), log);
    }

    @Test
    public void aRoutineMarksItsStepsInOrderAndRoutineEndLast() {
        installSink();
        runToEnd(Groups.sequential(
                StepMarks.wrap("first", body("one", 1)),
                StepMarks.wrap("second", body("two", 0)),
                StepMarks.routineEnd()));
        assertEquals(Arrays.asList(
                "mark first start @0", "one start @0", "one end NATURALLY @2", "mark first end @2",
                "mark second start @2", "two start @2", "two end NATURALLY @3", "mark second end @3",
                "mark routine-end end @3"), log);
    }

    /** A sequential of instant marks around each body would cost the routine loops at every step. */
    @Test
    public void markedStepsRunInTheSameLoopsAsTheirBodies() {
        runToEnd(Groups.sequential(body("x", 2), body("y", 0), body("z", 3)));
        List<String> bare = new ArrayList<>(log);
        int barePasses = pass;

        log.clear();
        Scheduler.reset();
        installSink();
        runToEnd(Groups.sequential(
                StepMarks.wrap("X", body("x", 2)),
                StepMarks.wrap("Y", body("y", 0)),
                StepMarks.wrap("Z", body("z", 3))));
        assertEquals(bare, entries(false));
        assertEquals(barePasses, pass);
        assertEquals(Arrays.asList(
                "mark X start @0", "mark X end @3", "mark Y start @3", "mark Y end @4",
                "mark Z start @4", "mark Z end @8"), entries(true));
    }

    @Test
    public void aStepHasItsBodysRequirementsPriorityAndBehaviours() {
        Object drivetrain = new Object();
        Command body = body("body", 1)
                .requiring(drivetrain)
                .setPriority(2)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setConflictBehavior(ConflictBehavior.QUEUE)
                .setBlockedBehavior(BlockedBehavior.QUEUE);
        Command step = StepMarks.wrap("drive", body);
        assertEquals(body.requirements(), step.requirements());
        assertEquals(2, step.priority());
        assertSame(InterruptedBehavior.SUSPEND, step.interruptedBehavior());
        assertSame(ConflictBehavior.QUEUE, step.conflictBehavior());
        assertSame(BlockedBehavior.QUEUE, step.blockedBehavior());

        Object intake = new Object();
        Command idle = Commands.infinite(() -> { }).requiring(intake).setEnd(c -> log.add("idle end " + c));
        schedule(idle);
        schedule(StepMarks.wrap("intake", body("take", 1).requiring(intake)));
        assertTrue(log.contains("idle end INTERRUPTED"));
    }

    @Test
    public void anInterruptedStepSendsNoEndMark() {
        installSink();
        Command step = StepMarks.wrap("hold", body("body", 1000));
        schedule(step);
        execute(1);
        step.cancel();
        assertEquals(Arrays.asList("mark hold start @0", "body start @0", "body end INTERRUPTED @1"), log);
    }

    @Test
    public void cancellingARoutineMarksNothingForStepsThatNeverEnded() {
        installSink();
        Command routine = Groups.sequential(
                StepMarks.wrap("a", body("a", 0)),
                StepMarks.wrap("b", body("b", 1000)),
                StepMarks.wrap("c", body("c", 0)),
                StepMarks.routineEnd());
        schedule(routine);
        execute(3);
        routine.cancel();
        assertEquals(Arrays.asList("mark a start @0", "mark a end @1", "mark b start @1"), entries(true));
    }

    @Test
    public void withNoSinkAStepRunsItsBodyAndMarksNothing() {
        runToEnd(StepMarks.wrap("drive", body("body", 2)));
        assertEquals(Arrays.asList("body start @0", "body end NATURALLY @2"), log);
        assertFalse(OpMode.blackboard.containsKey(StepMarks.SINK_KEY));
    }

    @Test
    public void aSinkThatIsNotABiConsumerIsIgnored() {
        Consumer<String> notASink = value -> log.add("wrong sink " + value);
        OpMode.blackboard.put(StepMarks.SINK_KEY, notASink);
        runToEnd(Groups.sequential(StepMarks.wrap("drive", body("body", 2)), StepMarks.routineEnd()));
        assertEquals(Arrays.asList("body start @0", "body end NATURALLY @3"), log);
        assertSame(notASink, OpMode.blackboard.get(StepMarks.SINK_KEY));
    }
}
