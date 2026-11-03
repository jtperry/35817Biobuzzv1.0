package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class LinearTableTest {

    private static final double EPS = 1e-9;

    @Test(expected = IllegalStateException.class)
    public void emptyTableThrows() {
        LinearTable table = new LinearTable();
        assertTrue(table.isEmpty());
        table.get(1.0);
    }

    @Test
    public void singlePointIsConstant() {
        LinearTable table = new LinearTable().put(10, 5);
        assertEquals(5.0, table.get(-100), EPS);
        assertEquals(5.0, table.get(100), EPS);
    }

    @Test
    public void blendsBetweenPointsAndClampsAtEnds() {
        LinearTable table = new LinearTable().put(10, 100).put(20, 200);
        assertEquals(150.0, table.get(15), EPS);
        assertEquals(100.0, table.get(10), EPS);
        assertEquals(100.0, table.get(0), EPS);
        assertEquals(200.0, table.get(25), EPS);
    }

    @Test
    public void insertOrderDoesNotMatterAndEqualKeysReplace() {
        LinearTable table = new LinearTable().put(30, 3).put(10, 1).put(20, 2).put(20, 2.5);
        assertEquals(3, table.size());
        assertEquals(10.0, table.keyAt(0), EPS);
        assertEquals(2.5, table.valueAt(1), EPS);
        assertEquals(1.75, table.get(15), EPS);
    }

    @Test
    public void clearEmptiesTheTable() {
        LinearTable table = new LinearTable().put(1, 1);
        table.clear();
        assertTrue(table.isEmpty());
    }

    @Test
    public void nanKeyReturnsTheFirstValueInsteadOfThrowing() {
        LinearTable table = new LinearTable().put(10, 100).put(20, 200);
        assertEquals(100.0, table.get(Double.NaN), EPS);
    }

    @Test
    public void interiorExactKeyReturnsItsOwnValue() {
        // 250 is off the straight line from (10, 100) to (30, 300), so a blend would give 200.
        LinearTable table = new LinearTable().put(10, 100).put(20, 250).put(30, 300);
        assertEquals(250.0, table.get(20), EPS);
    }

    @Test
    public void blendsInsideTheSecondSegment() {
        LinearTable table = new LinearTable().put(10, 100).put(20, 250).put(30, 300);
        assertEquals(275.0, table.get(25), EPS);
        assertEquals(175.0, table.get(15), EPS);
    }

    @Test(expected = IllegalStateException.class)
    public void clearedTableThrowsOnGet() {
        // A variant of emptyTableThrows: a table that had points and was cleared.
        LinearTable table = new LinearTable().put(10, 100);
        table.clear();
        table.get(10);
    }

    @Test
    public void putRejectsANonFiniteKey() {
        LinearTable table = new LinearTable();
        IllegalArgumentException nan = assertThrows(IllegalArgumentException.class, () -> table.put(Double.NaN, 1));
        assertTrue(nan.getMessage(), nan.getMessage().contains("NaN"));
        assertThrows(IllegalArgumentException.class, () -> table.put(Double.POSITIVE_INFINITY, 1));
        assertThrows(IllegalArgumentException.class, () -> table.put(Double.NEGATIVE_INFINITY, 1));
    }

    @Test
    public void putRejectsANonFiniteValue() {
        LinearTable table = new LinearTable();
        assertThrows(IllegalArgumentException.class, () -> table.put(10, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> table.put(10, Double.POSITIVE_INFINITY));
        assertThrows(IllegalArgumentException.class, () -> table.put(10, Double.NEGATIVE_INFINITY));
    }

    @Test
    public void aRejectedPointLeavesTheTableAsItWas() {
        LinearTable table = new LinearTable().put(10, 100);
        assertThrows(IllegalArgumentException.class, () -> table.put(Double.NaN, 5));
        assertThrows(IllegalArgumentException.class, () -> table.put(20, Double.POSITIVE_INFINITY));
        assertEquals(1, table.size());
        assertEquals(100.0, table.get(15), EPS);
    }
}
