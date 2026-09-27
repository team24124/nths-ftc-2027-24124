package org.firstinspires.ftc.teamcode.opmode.debug.tune;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.util.TelemetryControl;
import org.firstinspires.ftc.teamcode.util.Utilities;

import java.util.List;

@Config
@TeleOp(name = "FlyWheel", group = "tuning")
public class FlywheelDebugger extends OpMode {
    private GamepadEx driver;
    private List<LynxModule> hubs;
    private Flywheel flywheel;
    private Drivetrain drivetrain;
    private ActionScheduler actions;
    private TelemetryControl telemetryControl;
    private TeleOpTrajectories trajectories;
    public static double Kp = 0.004;
    public static double Kv = 0.00042;
    public static double velocity = 1; // Enter ticks/second

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        actions = ActionScheduler.INSTANCE;
        actions.init();
        driver = new GamepadEx(gamepad1);
        flywheel = new Flywheel(hardwareMap);
        telemetryControl = new TelemetryControl(telemetry);
        drivetrain = new Drivetrain(hardwareMap, new Pose(0, 0, 0)); // Start robot at the center of the field
        telemetryControl.subscribe(flywheel).subscribe(drivetrain);
        trajectories = TeleOpTrajectories.INSTANCE;
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        double y = driver.getLeftY();
        double x = driver.getLeftX();
        double rx = driver.getRightX();

        flywheel.setVelPID(Kp, Kv);

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            actions.schedule(flywheel.runFlywheel());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            actions.schedule(flywheel.stopFlywheel());
        }

        if (Utilities.isBetween(flywheel.wheel.getVelocity(), velocity - 50, velocity + 50)) {
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {

            }
        }

        if (flywheel.powered) {
            flywheel.power(velocity);
        } else {
            flywheel.wheel.setPower(0);
        }

        drivetrain.drive(x, y, rx, false);
        telemetryControl.getTelemetry().addData("Distance", trajectories.distanceToTarget(drivetrain, true));
        telemetryControl.getTelemetry().addData("vel", velocity);
        driver.readButtons();
        drivetrain.periodic();
        actions.run();
        telemetryControl.update();
    }
}
