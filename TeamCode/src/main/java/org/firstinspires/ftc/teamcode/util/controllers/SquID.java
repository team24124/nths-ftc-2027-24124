package org.firstinspires.ftc.teamcode.util.controllers;

import com.qualcomm.robotcore.util.Range;

/**
 * Implements a non-linear proportional controller using the square root of the error.
 * This provides high power for large errors and finer, smoother control as the
 * robot approaches the target, helping to reduce overshoot and oscillation.
 */
public class SquID {
    /*
     * Stretch coefficient that vertically expands the non-linear function and scales the overall power.
     * Tune this for responsiveness.
     * Lower stretch -> more power and responsiveness
     * Increasing stretch -> less power and overshoot
     */
    private double stretchFactor, tolerance;

    public SquID(double stretchFactor, double tolerance) {
        this.stretchFactor = stretchFactor;
        this.tolerance = tolerance;
    }

    public void setSquID(double stretchFactor, double tolerance) {
        this.stretchFactor = stretchFactor;
        this.tolerance = tolerance;
    }

    /**
     * Calculates the motor power based on the current position and target.
     *
     * @param position The current position of the motor (encoder ticks).
     * @param target The target position for the motor (encoder ticks).
     * @param voltage The current battery voltage, used for compensation.
     * @return The calculated motor power, clipped to the range [-1.0, 1.0].
     */
    public double calculate(double position, double target, double voltage) {
        // Remaining distance to target
        double error = target - position;
        if (Math.abs(error) < tolerance) {
            return 0.0;
        }

        // The core of the controller: power is proportional to the square root of the absolute error
        // Radical function where power is y (inverted below x = 0) and error is x
        double power = stretchFactor * Math.sqrt(Math.abs(error)) * Math.signum(error);
        power = Range.clip(power, -1.0, 1.0);

        // Voltage compensation. A lower bound of 8.0V prevents excessive scaling if the voltage reading is abnormal.
        return power * (12.0 / Math.max(voltage, 8.0));
    }
}