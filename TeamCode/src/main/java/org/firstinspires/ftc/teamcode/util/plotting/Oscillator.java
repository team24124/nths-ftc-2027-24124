package org.firstinspires.ftc.teamcode.util.plotting;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.ArraySelect;

public class Oscillator {
    private final ArraySelect<Double> setpoints;
    private double seconds;
    private boolean oscillate = false;
    private final ElapsedTime timer;

    public Oscillator(Double[] setpoints, double oscillationPeriodSeconds) {
        timer = new ElapsedTime();

        this.setpoints = new ArraySelect<>(setpoints);
        this.seconds = oscillationPeriodSeconds;
    }

    public double returnSetpoint() {
        double time = timer.seconds();
        if (oscillate) {
            if (time >= seconds) {
                timer.reset();
                if (setpoints.getSelectedIndex() + 1 == getSetpoints().length) {
                    return setpoints.moveSelection(-getSetpoints().length + 1).getSelected();
                }
                return setpoints.moveSelection(1).getSelected();
            } else {
                return setpoints.getSelected();
            }
        } else {
            return -1;
        }
    }

    public Double[] getSetpoints() {
        return setpoints.getAllOptions();
    }
    public boolean enabled() {
        return oscillate;
    }

    public void setOscillationPeriod(double seconds) {
        this.seconds = seconds;
    }

    public void enableOscillation(boolean enable) {
        oscillate = enable;
    }
}