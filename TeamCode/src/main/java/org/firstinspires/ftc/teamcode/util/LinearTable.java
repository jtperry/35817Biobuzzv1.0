package org.firstinspires.ftc.teamcode.util;

import java.util.Arrays;

/** Sorted key -> value points with straight-line blending between them. Keys past either end clamp to that end. */
public final class LinearTable {

    private double[] keys = new double[0];
    private double[] values = new double[0];

    /**
     * Add a point, or replace the value of an equal key. Returns this for chaining.
     * Throws for a NaN or infinite key or value.
     */
    public LinearTable put(double key, double value) {
        if (!Double.isFinite(key) || !Double.isFinite(value)) {
            throw new IllegalArgumentException("LinearTable point must be finite: (" + key + ", " + value + ")");
        }
        int index = Arrays.binarySearch(keys, key);
        if (index >= 0) {
            values[index] = value;
            return this;
        }
        int insert = -index - 1;
        double[] newKeys = new double[keys.length + 1];
        double[] newValues = new double[values.length + 1];
        System.arraycopy(keys, 0, newKeys, 0, insert);
        System.arraycopy(values, 0, newValues, 0, insert);
        newKeys[insert] = key;
        newValues[insert] = value;
        System.arraycopy(keys, insert, newKeys, insert + 1, keys.length - insert);
        System.arraycopy(values, insert, newValues, insert + 1, values.length - insert);
        keys = newKeys;
        values = newValues;
        return this;
    }

    public void clear() {
        keys = new double[0];
        values = new double[0];
    }

    public boolean isEmpty() {
        return keys.length == 0;
    }

    public int size() {
        return keys.length;
    }

    public double keyAt(int index) {
        return keys[index];
    }

    public double valueAt(int index) {
        return values[index];
    }

    /** Value at key, blended between the two nearest points, clamped at the ends. A NaN key returns the first point's value. */
    public double get(double key) {
        if (keys.length == 0) {
            throw new IllegalStateException("LinearTable is empty");
        }
        if (Double.isNaN(key)) {
            return values[0];
        }
        int last = keys.length - 1;
        if (key <= keys[0]) {
            return values[0];
        }
        if (key >= keys[last]) {
            return values[last];
        }
        int index = Arrays.binarySearch(keys, key);
        if (index >= 0) {
            return values[index];
        }
        int upper = -index - 1;
        int lower = upper - 1;
        double t = (key - keys[lower]) / (keys[upper] - keys[lower]);
        return values[lower] + (values[upper] - values[lower]) * t;
    }
}
