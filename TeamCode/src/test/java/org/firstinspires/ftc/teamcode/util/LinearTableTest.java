package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
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
}
