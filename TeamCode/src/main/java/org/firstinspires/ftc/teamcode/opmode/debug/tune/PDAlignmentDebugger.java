package org.firstinspires.ftc.teamcode.opmode.debug.tune;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;

import java.util.List;

@Config
@TeleOp(name = "Align PD", group = "tuning")
public class PDAlignmentDebugger extends OpMode {
    private List<LynxModule> hubs;
    private VoltageSensor voltageSensor;
    private Drivetrain drivetrain;
    private TeleOpTrajectories trajectories;
    private GamepadEx driver;

    // --- Tune PD ---
    public static double Kp = 7;
    public static double Kd = 0.7;
    public static double sf = 0.7;
    public static double targetX = 72;
    public static double targetY = -72;

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
        drivetrain = new FieldCentricDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        trajectories = TeleOpTrajectories.INSTANCE;
        driver = new GamepadEx(gamepad1);
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
            alignToAT = !alignToAT;
        }

        if (driver.wasJustPressed(GamepadKeys.Button.START)) {
            Vector2d current = drivetrain.getDrive().localizer.getPose().position;
            drivetrain.getDrive().localizer.setPose(new Pose2d(current, 0));
        }

        if (alignToAT) {
            double rotation = pd.calculate(-trajectories.theta(drivetrain, targetX, targetY), 0, voltageSensor.getVoltage());
            drivetrain.drive(x, y, rotation, false);
        } else {
            drivetrain.drive(x, y, rx, false);
        }
        drivetrain.periodic();

        driver.readButtons();

        telemetry.addData("\nAlign", alignToAT);
        telemetry.addData("\nX", "%.1f", drivetrain.getPosition().component1().x);
        telemetry.addData("Y", "%.1f", drivetrain.getPosition().component1().y);
        telemetry.addData("\nHeading", "%.1f", drivetrain.getPosition().heading.toDouble());
        telemetry.addData("\nTheta to target", "%.2f", trajectories.theta(drivetrain, targetX, targetY));
        telemetry.update();
    }
}