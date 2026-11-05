package org.firstinspires.ftc.teamcode.tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class ShotTunerSessionTest {

    private static final double EPS = 1e-9;
    private static final Set<Double> NONE = Collections.<Double>emptySet();

    private static ShotTunerLane.Spot spot(double nominalIn, boolean valid) {
        return new ShotTunerLane.Spot(nominalIn, 0, 0, 0, valid, valid ? "" : "wall");
    }

    private static List<ShotTunerLane.Spot> lane(ShotTunerLane.Spot... spots) {
        return Arrays.asList(spots);
    }

    private static Set<Double> saved(Double... nominals) {
        return new HashSet<>(Arrays.asList(nominals));
    }

    private static void assertAt(ShotTunerSession session, int index, double nominalIn) {
        assertFalse(session.done());
        assertEquals(index, session.index());
        assertEquals(nominalIn, session.current().nominalIn, EPS);
        assertTrue(session.current().valid);
    }

    private static void assertDone(ShotTunerSession session) {
        assertTrue(session.done());
        assertEquals(-1, session.index());
        assertNull(session.current());
    }

    @Test
    public void resumeStartsAtTheFirstValidUnsavedSpot() {
        ShotTunerSession session = new ShotTunerSession(
                lane(spot(30, false), spot(35, true), spot(40, true), spot(45, true)), saved(35.0));
        assertAt(session, 2, 40);
    }

    @Test
    public void withNothingSavedItStartsAtTheFirstValidSpot() {
        ShotTunerSession session = new ShotTunerSession(
                lane(spot(30, false), spot(35, true), spot(40, true)), NONE);
        assertAt(session, 1, 35);
    }

    @Test
    public void invalidSpotsAreNeverCurrent() {
        // NONE is unmodifiable: a session that wrote to the caller's set would throw here.
        ShotTunerSession session = new ShotTunerSession(
                lane(spot(30, false), spot(35, true), spot(40, false), spot(45, true), spot(50, false)), NONE);
        assertAt(session, 1, 35);
        session.save();
        assertAt(session, 3, 45);
        session.back();
        assertAt(session, 1, 35);
        session.skip();
        assertAt(session, 3, 45);
        session.save();
        assertDone(session);
        session.back();
        assertAt(session, 3, 45);
    }

    @Test
    public void saveMarksTheSpotAndMovesToTheNextValidUnsavedSpot() {
        ShotTunerSession session = new ShotTunerSession(
                lane(spot(30, true), spot(35, true), spot(40, true), spot(45, true)), saved(40.0));
        assertAt(session, 0, 30);
        session.save();
        assertTrue(session.isSaved(30));
        assertAt(session, 1, 35);
        session.save();
        assertAt(session, 3, 45);
        assertEquals(3, session.savedCount());
    }

    @Test
    public void skipMovesOnWithoutSaving() {
        ShotTunerSession session = new ShotTunerSession(
                lane(spot(30, true), spot(35, true), spot(40, true)), NONE);
        session.skip();
        assertAt(session, 1, 35);
        assertFalse(session.isSaved(30));
        assertEquals(0, session.savedCount());
    }

    @Test
    public void backGoesToThePreviousValidSpotEvenIfSavedAndReSaveOverwrites() {
        ShotTunerSession session = new ShotTunerSession(
                lane(spot(30, true), spot(35, false), spot(40, true), spot(45, true)), NONE);
        session.save();
        assertAt(session, 2, 40);
        session.back();
        assertAt(session, 0, 30);
        assertTrue(session.isSaved(30));
        session.save();
        assertAt(session, 2, 40);
        assertEquals(1, session.savedCount());
    }

    @Test
    public void skipAfterBackToSavedSpotsGoesToTheNextUnsavedSpot() {
        ShotTunerSession session = new ShotTunerSession(
                lane(spot(30, true), spot(35, true), spot(40, true), spot(45, true)), saved(30.0, 35.0));
        assertAt(session, 2, 40);
        session.back();
        assertAt(session, 1, 35);
        session.back();
        assertAt(session, 0, 30);
        session.skip();
        assertAt(session, 2, 40);
    }

    @Test
    public void backAtTheFirstValidSpotStaysPut() {
        ShotTunerSession session = new ShotTunerSession(
                lane(spot(30, false), spot(35, true), spot(40, true)), NONE);
        session.back();
        assertAt(session, 1, 35);
    }

    @Test
    public void backFromDoneGoesToTheLastValidSpot() {
        ShotTunerSession session = new ShotTunerSession(
                lane(spot(30, true), spot(35, true), spot(40, false)), NONE);
        session.save();
        session.save();
        assertDone(session);
        session.back();
        assertAt(session, 1, 35);
        assertTrue(session.isSaved(35));
        session.skip();
        assertDone(session);
    }

    @Test
    public void doneWhenNoValidUnsavedSpotRemainsAheadAndANewSessionResumesAtTheSkippedSpot() {
        List<ShotTunerLane.Spot> spots = lane(spot(30, true), spot(35, true), spot(40, true), spot(45, true));
        ShotTunerSession session = new ShotTunerSession(spots, NONE);
        session.skip();
        session.save();
        session.save();
        session.save();
        assertDone(session);
        assertEquals(3, session.savedCount());
        assertEquals(4, session.validCount());
        session.save();
        session.skip();
        assertDone(session);
        assertEquals(3, session.savedCount());

        ShotTunerSession resumed = new ShotTunerSession(spots, saved(35.0, 40.0, 45.0));
        assertAt(resumed, 0, 30);
    }

    @Test
    public void everyValidSpotSavedStartsDone() {
        ShotTunerSession session = new ShotTunerSession(
                lane(spot(30, true), spot(35, true), spot(40, false)), saved(30.0, 35.0));
        assertDone(session);
        assertEquals(2, session.savedCount());
        assertEquals(2, session.validCount());
    }

    @Test
    public void noValidSpotsIsDoneAndBackStaysDone() {
        ShotTunerSession invalid = new ShotTunerSession(lane(spot(30, false), spot(35, false)), NONE);
        assertDone(invalid);
        assertEquals(0, invalid.validCount());
        invalid.back();
        assertDone(invalid);

        ShotTunerSession empty = new ShotTunerSession(lane(), NONE);
        assertDone(empty);
        assertEquals(0, empty.validCount());
        assertEquals(0, empty.savedCount());
        empty.back();
        assertDone(empty);
    }

    @Test
    public void countsOnlyValidSavedSpotsAndMatchesNominalsWithinTolerance() {
        ShotTunerSession session = new ShotTunerSession(
                lane(spot(30, true), spot(35, false), spot(40, true), spot(45, true)),
                saved(35.0, 45.0000001, 99.0));
        assertEquals(3, session.validCount());
        assertEquals(1, session.savedCount());
        assertTrue(session.isSaved(45));
        assertTrue(session.isSaved(35));
        assertFalse(session.isSaved(45.001));
        assertFalse(session.isSaved(40));
        assertAt(session, 0, 30);
        session.save();
        session.save();
        assertDone(session);
        assertEquals(3, session.savedCount());
    }

    @Test
    public void copiesTheCallersSpotListAndSavedSet() {
        List<ShotTunerLane.Spot> spots = new ArrayList<>(
                lane(spot(30, true), spot(35, true), spot(40, true)));
        Set<Double> callerSaved = saved(30.0);
        ShotTunerSession session = new ShotTunerSession(spots, callerSaved);
        assertAt(session, 1, 35);
        session.save();
        assertTrue(session.isSaved(35));
        assertEquals(1, callerSaved.size());
        assertFalse(callerSaved.contains(35.0));
        spots.add(spot(45, true));
        assertEquals(3, session.validCount());
        assertAt(session, 2, 40);
    }
}
