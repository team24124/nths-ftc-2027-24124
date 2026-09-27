package org.firstinspires.ftc.teamcode.util;

public final class Utilities {
    /**
     * Determine whether a given value is between the ranges of a given min and max
     *
     * @param value The value
     * @param min   The lower bound of the range
     * @param max   The upper bound of the range
     * @return True, if the value is between the min and max, false otherwise
     */
    public static boolean isBetween(int value, double min, double max) {
        if (min > max) throw new IllegalArgumentException("The minimum value you attempted to compare is greater than your maximum value.");
        return value >= min && value <= max;
    }

    /**
     * Determine whether a given value is between the ranges of a given min and max
     *
     * @param value The value
     * @param min The lower bound of the range
     * @param max The upper bound of the range
     * @return True, if the value is between the min and max, false otherwise
     */
    public static boolean isBetween(double value, double min, double max){
        if(min > max) throw new IllegalArgumentException("The minimum value you attempted to compare is greater than your maximum value.");
        return value >= min && value <= max;
    }

    /**
     * Calculate and return the percent error between two given values
     *
     * @param theoretical Theoretical maximumum yield
     * @param actual      Actual yield
     * @return A double (theoretical / actual)
     */
    public static double calculateErr(double theoretical, double actual) {
        return (actual / theoretical);
    }
}
