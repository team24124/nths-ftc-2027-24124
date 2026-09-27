package org.firstinspires.ftc.teamcode.opmode.debug.tune;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;

import java.util.List;

@Config
@TeleOp(name = "Align LL", group = "tuning")
public class LLAlignmentDebugger extends OpMode {
    private List<LynxModule> hubs;
    private VoltageSensor voltageSensor;
    private Drivetrain drivetrain;
    private TeleOpTrajectories trajectories;
    private Limelight limelight;
    private GamepadEx driver;

    // --- Tune PD ---
    public static double Kp = 3;
    public static double Kd = 0.1;
    public static double sf = 0.7;

    // --- PD ---
    private boolean alignToAT = false;
    private PIDF pd = new PIDF();

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        drivetrain = new Drivetrain(hardwareMap, new Pose(0, 0, 0));
        trajectories = TeleOpTrajectories.INSTANCE;
        driver = new GamepadEx(gamepad1);
        limelight = new Limelight(hardwareMap);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        pd.setPD(Kp, Kd, sf);

        double y = driver.getLeftY();
        double x = driver.getLeftX();
        double rx = driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            limelight.setPipeline(Limelight.Pipeline.AT1);
            alignToAT = true;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            alignToAT = false;
        }

        if (driver.wasJustPressed(GamepadKeys.Button.START)) {
            Pose current = drivetrain.getPosition();
            drivetrain.getDrivetrain().setPose(new Pose(current.x(), current.y(), 0));
        }

        if (alignToAT) {
            if (limelight.isDetected()) {
                drivetrain.drive(x, y, -pd.calculate(Math.toRadians(limelight.degreeOffset()), 0, voltageSensor.getVoltage()), false);
            } else {
                drivetrain.drive(x, y, -pd.calculate(trajectories.theta(drivetrain, 72, 72), 0, voltageSensor.getVoltage()), false);
            }
        } else {
            drivetrain.drive(x, y, rx, false);
        }
        drivetrain.periodic();

        driver.readButtons();

        telemetry.addData("\nAlign", alignToAT);
        telemetry.addData("\nIs Detected", limelight.isDetected());
        telemetry.addData("\nX", "%.1f", drivetrain.getPosition().component1().x);
        telemetry.addData("Y", "%.1f", drivetrain.getPosition().component1().y);
        telemetry.addData("\nHeading", "%.1f", drivetrain.getPosition().heading.toDouble());
        telemetry.update();
    }
}