package org.firstinspires.ftc.teamcode.tuning;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Shot Tuner progress over the lane spots (spec 7.3): which spot is current and which are saved.
 * Invalid spots are never current. Save and skip move forward to the next valid unsaved spot (no
 * wrap-around: a skipped spot comes back on the next resume). Back steps to the previous valid spot,
 * saved or not, so a re-save overwrites it.
 */
public final class ShotTunerSession {

    /** Saved nominals come back from the CSV, so they match a spot within this, inches. */
    private static final double MATCH_IN = 1e-6;

    private final List<ShotTunerLane.Spot> spots;
    private final List<Double> saved;
    private int index;

    /** Resumes at the first valid spot not in savedNominals; done if there is none. */
    public ShotTunerSession(List<ShotTunerLane.Spot> spots, Set<Double> savedNominals) {
        this.spots = new ArrayList<>(spots);
        this.saved = new ArrayList<>(savedNominals);
        index = nextUnsaved(-1);
    }

    /** The spot to tune, or null when done. */
    public ShotTunerLane.Spot current() {
        return index < 0 ? null : spots.get(index);
    }

    /** Position of current() in the spot list (invalid spots count), -1 when done. */
    public int index() {
        return index;
    }

    public boolean done() {
        return index < 0;
    }

    /** Marks the current spot saved and moves to the next valid unsaved spot after it. */
    public void save() {
        if (index < 0) {
            return;
        }
        double nominal = spots.get(index).nominalIn;
        if (!isSaved(nominal)) {
            saved.add(nominal);
        }
        index = nextUnsaved(index);
    }

    /** Moves to the next valid unsaved spot without saving. */
    public void skip() {
        if (index < 0) {
            return;
        }
        index = nextUnsaved(index);
    }

    /** Previous valid spot, saved or not; from done, the last valid spot. Stays put at the first one. */
    public void back() {
        int from = index < 0 ? spots.size() : index;
        for (int i = from - 1; i >= 0; i--) {
            if (spots.get(i).valid) {
                index = i;
                return;
            }
        }
    }

    public boolean isSaved(double nominalIn) {
        for (double nominal : saved) {
            if (Math.abs(nominal - nominalIn) <= MATCH_IN) {
                return true;
            }
        }
        return false;
    }

    /** Valid spots that are saved. A saved nominal that is not a valid spot here does not count. */
    public int savedCount() {
        int count = 0;
        for (ShotTunerLane.Spot spot : spots) {
            if (spot.valid && isSaved(spot.nominalIn)) {
                count++;
            }
        }
        return count;
    }

    public int validCount() {
        int count = 0;
        for (ShotTunerLane.Spot spot : spots) {
            if (spot.valid) {
                count++;
            }
        }
        return count;
    }

    private int nextUnsaved(int after) {
        for (int i = after + 1; i < spots.size(); i++) {
            ShotTunerLane.Spot spot = spots.get(i);
            if (spot.valid && !isSaved(spot.nominalIn)) {
                return i;
            }
        }
        return -1;
    }
}
