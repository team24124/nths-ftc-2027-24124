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
    private final ArraySelect<Double> speeds;
    public PIDF thetaPD = new PIDF();
    public final VoltageSensor voltageSensor;

    public Drivetrain(HardwareMap hw, Pose start) {
        drivetrain = Constants.create(hw);
        drivetrain.setPose(start);

        speeds = new ArraySelect<>(new Double[]{0.5, 1.0});

        thetaPD.setPD(4,0.25,0.7);

        voltageSensor = hw.get(VoltageSensor.class, "Control Hub");
    }

    public Follower getDrivetrain(){
        return drivetrain;
    }

    public ArraySelect<Double> getSpeeds() {
        return speeds;
    }

    public void toggleSpeeds() {
        speeds.setSelected(Math.abs(speeds.getSelectedIndex() - 1));
    }

    public Pose getPosition() {
        return drivetrain.pose();
    }

    public void drive(double x, double y, double rx, boolean align){
        if (align) {
            if (getSpeeds().getSelected() == 1.0) {
                thetaPD.setPD(1.4,0.045,0);
            } else {
                thetaPD.setPD(4,0.25,0);
            }
            rx = -thetaPD.calculate(rx, 0, voltageSensor.getVoltage()); // rx is limelight/theta from odometry input in this case due to different input parameters in main TeleOps
        }

        DrivePowers powers = ManualDrive.fieldCentric(
                y * speeds.getSelected(),
                x * speeds.getSelected(),
                rx * speeds.getSelected(),
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
        telemetry.addData("Speed", getSpeeds().getSelected());
        telemetry.addData("Heading (rad)", drivetrain.pose().heading());
        telemetry.addData("Heading (°)", Math.toDegrees(drivetrain.pose().heading()));
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }
}
