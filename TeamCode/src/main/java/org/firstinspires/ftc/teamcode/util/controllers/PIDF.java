package org.firstinspires.ftc.teamcode.util.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF {
    private double Kp, Ki, Kd, Kg, Kv, Ka, Ks = 0; // Proportional, integral, derivative, gravitational correction ff, velocity ff, acceleration ff
    private double smoothingFactor = 0; // sf can be any value from 0 < sf < 1. Heavier smoothing but less responsiveness towards 1
    private double integralSumLimit = 0; // Integral cap to prevent unnecessary accumulation

    private double lastTarget, integralSum, prevError, previousFilterEstimate = 0;
    private double motorTPR = 1;
    private boolean resetIntegral = false;

    private ElapsedTime timer = new ElapsedTime();

    public void setPIDG(double Kp, double Ki, double Kd, double Kg, double smoothingFactor, double integralSumLimit, double motorTPR) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kg = Kg;
        this.smoothingFactor = smoothingFactor;
        this.integralSumLimit = integralSumLimit;
        this.motorTPR = motorTPR;
    }
    public void setPIDV(double Kp, double Ki, double Kd, double Kv, double smoothingFactor, double integralSumLimit) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kv = Kv;
        this.smoothingFactor = smoothingFactor;
        this.integralSumLimit = integralSumLimit;
    }
    public void setPID(double Kp, double Ki, double Kd, double smoothingFactor, double integralSumLimit) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.smoothingFactor = smoothingFactor;
        this.integralSumLimit = integralSumLimit;
    }
    public void setPD(double Kp, double Kd, double smoothingFactor) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.smoothingFactor = smoothingFactor;
    }
    public void setPV(double Kp, double Kv) {
        this.Kp = Kp;
        this.Kv = Kv;
    }

    public void enableFilters(boolean resetIntegral) {
        this.resetIntegral = resetIntegral;
    }

    // Outputs processed power
    public double calculate(double position, double target, double voltage) {
        // Time
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        // Error
        double error = target - position;

        // Integral setters & limiters (optional resetting)
        if (resetIntegral && lastTarget != target) {
            integralSum = 0;
            lastTarget = target;
        }
        integralSum += (error * dt);
        if (Math.abs(integralSum) > integralSumLimit) {
            integralSum = integralSumLimit * Math.signum(integralSum);
        }

        // Derivative setter & sensor noise filters
        double rawDerivative = (error - prevError) / dt;
        double derivative = (smoothingFactor * previousFilterEstimate) + (1 - smoothingFactor) * rawDerivative;
        previousFilterEstimate = derivative;

        // Angle getter
        double radians = 2 * Math.PI * (position / motorTPR); // Assumes an arm is initialized horizontally, TPR is gear ratio * 28, goBILDA 312rpm 5203 yellowjacket motor

        // Output calculator
        double out = ((Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Kg * Math.cos(radians)) + (Kv * target)) * (12.0 / Math.max(voltage, 8.0));
        out = Math.max(-1, Math.min(1, out));

        // Output & error setter
        prevError = error;
        return out;
    }
}