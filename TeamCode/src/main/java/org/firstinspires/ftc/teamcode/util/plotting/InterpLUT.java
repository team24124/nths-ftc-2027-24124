package org.firstinspires.ftc.teamcode.util.plotting;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Simple piecewise-linear lookup table with interpolation.
 * Keys (xs) must be unique.
 *
 * Usage:
 *  - new InterpolatedLookupTable(xs, ys)
 *  - put(x,y) to insert (keeps sorted)
 *  - get(x) to retrieve interpolated y
 * OR
 *  - double[] dists = {5, 10, 20, 40};
 *  - double[] powers = {0.8, 0.6, 0.3, 0.15};
 *  - InterpolatedLookupTable table = new InterpolatedLookupTable(dists, powers);
 *
 * Extrapolation modes: CLAMP (use nearest y) or LINEAR (extend segment).
 *
 */

public class InterpLUT {
    public enum Extrapolation { CLAMP, LINEAR }

    private final List<Double> xs = new ArrayList<>();
    private final List<Double> ys = new ArrayList<>();
    private Extrapolation extrapolation = Extrapolation.CLAMP;

    public InterpLUT() {}

    /** Construct with arrays (must be same length). They do not need to be sorted; constructor sorts by x. */
    public InterpLUT(double[] xArr, double[] yArr) {
        if (xArr == null || yArr == null) throw new IllegalArgumentException("arrays required");
        if (xArr.length != yArr.length) throw new IllegalArgumentException("arrays must be same length");
        for (int i = 0; i < xArr.length; i++) {
            put(xArr[i], yArr[i]);
        }
    }

    public void setExtrapolation(Extrapolation mode) { this.extrapolation = mode; }
    public Extrapolation getExtrapolation() { return this.extrapolation; }

    /** Insert or replace a point (keeps xs sorted). If x exists, replace y. */
    public void put(double x, double y) {
        int idx = Collections.binarySearch(xs, x);
        if (idx >= 0) {
            ys.set(idx, y); // replace
        } else {
            int insert = -idx - 1;
            xs.add(insert, x);
            ys.add(insert, y);
        }
    }

    /** Clear the table */
    public void clear() {
        xs.clear();
        ys.clear();
    }

    public int size() {
        return xs.size();
    }

    /** Return interpolated y for given x. */
    public double get(double x) {
        int n = xs.size();
        if (n == 0) throw new IllegalStateException("Lookup table is empty");

        // short-circuit exact ends
        if (n == 1) return ys.get(0);

        // binary search for insertion point / exact match
        int idx = Collections.binarySearch(xs, x);
        if (idx >= 0) {
            return ys.get(idx); // exact key found
        }

        int insert = -idx - 1;

        // x is before first key
        if (insert == 0) {
            if (extrapolation == Extrapolation.CLAMP) {
                return ys.get(0);
            } else { // LINEAR
                return linearInterp(x, xs.get(0), ys.get(0), xs.get(1), ys.get(1));
            }
        }

        // x is after last key
        if (insert >= n) {
            if (extrapolation == Extrapolation.CLAMP) {
                return ys.get(n - 1);
            } else {
                return linearInterp(x, xs.get(n - 2), ys.get(n - 2), xs.get(n - 1), ys.get(n - 1));
            }
        }

        // Between insert-1 and insert
        double x0 = xs.get(insert - 1);
        double y0 = ys.get(insert - 1);
        double x1 = xs.get(insert);
        double y1 = ys.get(insert);

        return linearInterp(x, x0, y0, x1, y1);
    }

    /** Linear interpolation between (x0,y0) and (x1,y1) */
    private double linearInterp(double x, double x0, double y0, double x1, double y1) {
        if (x1 == x0) return y0; // degenerate, avoid div by zero
        double t = (x - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    public double getKey(int index) { return xs.get(index); }
    public double getValue(int index) { return ys.get(index); }

    /** Returns a copy of keys */
    public double[] keys() {
        double[] a = new double[xs.size()];
        for (int i = 0; i < xs.size(); i++) a[i] = xs.get(i);
        return a;
    }

    /** Returns a copy of values */
    public double[] values() {
        double[] a = new double[ys.size()];
        for (int i = 0; i < ys.size(); i++) a[i] = ys.get(i);
        return a;
    }
}
