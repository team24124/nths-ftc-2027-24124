package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;

public class Flywheel implements SubsystemBase, TelemetryObservable {
    public final DcMotorEx wheel;
    public final Servo gate;
    public boolean powered = false;
    public boolean primed = false;
    public PIDF pv = new PIDF();
    //double[] dists = {40, 50, 60, 70, 80, 90, 100, 150}; // Inches
    //double[] vels = {1030, 1040, 1080, 1110, 1190, 1250, 1390, 1480}; // Ticks/second max 2800
    public final VoltageSensor voltageSensor;
    //public InterpLUT lut = new InterpLUT(dists, vels);
    private double distance = 1;
    //public double targetVel = lut.get(distance);
    public double targetVel = 900;

    public Flywheel(HardwareMap hw) {
        wheel = hw.get(DcMotorEx.class, "wheel");
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gate = hw.get(Servo.class, "gate");

        voltageSensor = hw.get(VoltageSensor.class, "Control Hub");

        pv.setPV(0.004, 0.00042);

    //    lut.setExtrapolation(InterpLUT.Extrapolation.LINEAR);
    }

    /**
     * Distance(Inches) = Distance to goal
     * Velocity(ticks/s) = tps of 5203 YellowJacket 6k rpm motor (1:1 GR, 1 x 28 tpr)
     */
    @Override
    public void periodic(){
        //targetVel = lut.get(distance);
        if (powered) {
            power(targetVel);
            primed = Utilities.isBetween(wheel.getVelocity(), targetVel -100, targetVel + 50);
        } else {
            wheel.setPower(0);
            primed = false;
        }
    }

    public Command autonPeriodic() {
        return Command.build()
                .setExecute(this::periodic);
    }

    public Command runFlywheel() {
        return instant(() -> powered = true);
    }

    public Command stopFlywheel() {
        return instant(() -> {powered = false; primed = false;});
    }

    public Command setVls(double d) {
        return instant(() -> distance = d);
    }

    public void power(double vel) {
        double power = pv.calculate(-wheel.getVelocity(), vel, voltageSensor.getVoltage());
        wheel.setPower(power);
    }

    public void setVelPID(double Kp, double Kv) {
        pv.setPV(Kp, Kv);
    }

    public Command openGate() {
        return instant(() -> {
            gate.setPosition(0.5);
        });
    }

    public Command closeGate() {
        return instant(() -> {
            gate.setPosition(0);
        });
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Flywheel Powered", powered);
    //    telemetry.addData("Flywheel LUT Value", lut.get(distance));
        telemetry.addData("Flywheel Target Velocity (tps)", targetVel);
        telemetry.addData("Flywheel Velocity (tps)", wheel.getVelocity());
        telemetry.addData("Flywheel Primed", primed);
        telemetry.addData("Servo State", gate.getPosition());
    }

    @Override
    public String getName() {
        return "Flywheel";
    }
}