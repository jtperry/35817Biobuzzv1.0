package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.function.BiConsumer;

/**
 * Where each auto routine step starts and ends, for the laptop sim. Marks go to the
 * {@code java.util.function.BiConsumer<String, String>} (step id, event) the sim puts in OpMode.blackboard under
 * SINK_KEY: a JDK type, so it crosses the sim's class loaders. On the robot there is no sink: a mark is one lookup.
 */
public final class StepMarks {

    public static final String SINK_KEY = "biobuzz.sim.steps";
    public static final String START = "start";
    public static final String END = "end";
    /** Marked once, with END, after the routine's last step: the auto is done. */
    public static final String ROUTINE_END = "routine-end";

    private StepMarks() {
    }

    /**
     * body, marked START as it starts and END when it ends on its own (not when interrupted). Otherwise it is body:
     * same requirements, priority and behaviours, same loops. Not a sequential of instant marks around body: Ivy's
     * Sequential takes a loop to move past each child, so that would cost the routine about 3 loops per step.
     */
    static Command wrap(String id, Command body) {
        return Command.build()
                .requiring(body.requirements())
                .setPriority(body.priority())
                .setInterruptedBehavior(body.interruptedBehavior())
                .setConflictBehavior(body.conflictBehavior())
                .setBlockedBehavior(body.blockedBehavior())
                .setStart(() -> {
                    mark(id, START);
                    body.start();
                })
                .setExecute(body::execute)
                .setDone(body::done)
                .setEnd(condition -> {
                    body.end(condition);
                    if (condition == EndCondition.NATURALLY) {
                        mark(id, END);
                    }
                });
    }

    /** The routine's last element: marks ROUTINE_END. */
    static Command routineEnd() {
        return Commands.instant(() -> mark(ROUTINE_END, END));
    }

    /** (id, event) to the sim's sink, if the blackboard holds one. */
    @SuppressWarnings("unchecked")
    static void mark(String id, String event) {
        Object sink = OpMode.blackboard.get(SINK_KEY);
        if (sink instanceof BiConsumer) {
            ((BiConsumer<String, String>) sink).accept(id, event);
        }
    }
}
