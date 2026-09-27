package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;
import org.firstinspires.ftc.teamcode.util.plotting.InterpLUT;

public class Flywheel implements SubsystemBase, TelemetryObservable {
    public final DcMotorEx wheel;
    public boolean powered = false;
    public boolean primed = false;
    public PIDF pv = new PIDF();
    double[] dists = {40, 50, 60, 70, 80, 90, 100, 150}; // Inches
    double[] vels = {1030, 1040, 1080, 1110, 1190, 1250, 1390, 1480}; // Ticks/second max 2800
    public final VoltageSensor voltageSensor;
    public InterpLUT lut = new InterpLUT(dists, vels);
    private double distance = 1;
    public double targetVel = lut.get(distance);

    public Flywheel(HardwareMap hw) {
        wheel = hw.get(DcMotorEx.class, "wheel"); // Connected Ehub 2
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheel.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hw.get(VoltageSensor.class, "Control Hub");

        pv.setPV(0.004, 0.00042);

        lut.setExtrapolation(InterpLUT.Extrapolation.LINEAR);
    }

    /**
     * Distance(Inches) = Distance to goal
     * Velocity(ticks/s) = tps of 5203 YellowJacket 6k rpm motor (1:1 GR, 1 x 28 tpr)
     */
    @Override
    public void periodic(){
        targetVel = lut.get(distance);
        if (powered) {
            power(targetVel);
            primed = Utilities.isBetween(wheel.getVelocity(), targetVel -175, targetVel + 50);
        } else {
            wheel.setPower(0);
            primed = false;
        }
    }

    public Action autonPeriodic() {
        return (TelemetryPacket packet) -> {
            periodic();

            return true;
        };
    }

    public Action runFlywheel() {
        return (TelemetryPacket packet) -> {
            powered = true;

            return false;
        };
    }

    public Action stopFlywheel() {
        return (TelemetryPacket packet) -> {
            powered = false;
            primed = false;

            return false;
        };
    }

    public void power(double vel) {
        double power = pv.calculate(-wheel.getVelocity(), vel, voltageSensor.getVoltage());
        wheel.setPower(power);
    }

    public Action setVls(double d) {
        return (TelemetryPacket packet) -> {
            distance = d;
            return false;
        };
    }

    public void setVelPID(double Kp, double Kv) {
        pv.setPV(Kp, Kv);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Flywheel Powered", powered);
        telemetry.addData("Flywheel LUT Value", lut.get(distance));
        telemetry.addData("Flywheel Target Velocity (tps)", targetVel);
        telemetry.addData("Flywheel Velocity (tps)", wheel.getVelocity());
        telemetry.addData("Flywheel Primed", primed);
    }

    @Override
    public String getName() {
        return "Flywheel";
    }
}