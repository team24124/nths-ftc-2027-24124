package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.TelemetryControl;

public class Robot {
    public Intake intake;
    public Flywheel flywheel;
    public Drivetrain drivetrain;
    public Limelight limelight;
    public TelemetryControl telemetryControl;

    public Robot(HardwareMap hw, Telemetry telemetry, boolean robotCentric) {
        intake = new Intake(hw);
        flywheel = new Flywheel(hw);
        drivetrain = new Drivetrain(hw, PoseStorage.currentPose);
        limelight = new Limelight(hw);

        telemetryControl = new TelemetryControl(telemetry);
        telemetryControl
                .subscribe(intake)
                .subscribe(flywheel)
                .subscribe(drivetrain)
                .subscribe(limelight);
    }
}