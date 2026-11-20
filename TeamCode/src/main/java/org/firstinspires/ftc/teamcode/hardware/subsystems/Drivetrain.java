package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.ArraySelect;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;

public class Drivetrain implements SubsystemBase, TelemetryObservable {
    private final Follower drivetrain;
    public PIDF thetaPDF = new PIDF();
    public final VoltageSensor voltageSensor;

    public Drivetrain(HardwareMap hw, Pose start) {
        drivetrain = Constants.create(hw);
        drivetrain.setPose(start);

        thetaPDF.setPDF(3,0.2, 0.5,0);

        voltageSensor = hw.get(VoltageSensor.class, "Control Hub");
    }

    public Follower getDrivetrain(){
        return drivetrain;
    }

    public Pose getPosition() {
        return drivetrain.pose();
    }

    public void drive(double x, double y, double rx, boolean align, double d, double FFVal){
        if (align) {
            // not zero alignment is a temporary measure
            rx = thetaPDF.calculate(rx, -Math.atan(3.043/d), FFVal, voltageSensor.getVoltage()); // rx is limelight/theta from odometry input in this case due to different input parameters in main TeleOps
        }

        DrivePowers powers = ManualDrive.fieldCentric(
                y, x, rx,
                drivetrain.pose().heading()
        );

        ManualDrive.driveOrHold(drivetrain, powers);
        drivetrain.update();
    }

    @Override
    public void periodic() {
        getDrivetrain().update();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("X", drivetrain.pose().x());
        telemetry.addData("Y", drivetrain.pose().y());
        telemetry.addData("Heading (rad)", drivetrain.pose().heading());
        telemetry.addData("Heading (°)", Math.toDegrees(drivetrain.pose().heading()));
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }
}
