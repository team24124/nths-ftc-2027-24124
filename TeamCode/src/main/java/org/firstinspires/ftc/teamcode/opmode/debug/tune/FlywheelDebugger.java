package org.firstinspires.ftc.teamcode.opmode.debug.tune;

import static com.pedropathing.ivy.groups.Groups.parallel;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
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
    private Intake intake;
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

        Scheduler.reset();
        driver = new GamepadEx(gamepad1);
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);
        telemetryControl = new TelemetryControl(telemetry);
        drivetrain = new Drivetrain(hardwareMap, new Pose(0, 0, 0)); // Start robot at the center of the field facing +X
        telemetryControl.subscribe(flywheel).subscribe(drivetrain).subscribe(intake);
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
            Scheduler.schedule(flywheel.runFlywheel());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            Scheduler.schedule(flywheel.stopFlywheel());
        }

        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            Scheduler.schedule(parallel(flywheel.openGate()), intake.runIntake());
        }
        if (driver.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            Scheduler.schedule(parallel(flywheel.closeGate()), intake.stopIntake());
        }

        if (flywheel.powered) {
            flywheel.power(velocity);
        } else {
            flywheel.wheel.setPower(0);
        }

        drivetrain.drive(x, y, rx, false);
        telemetryControl.getTelemetry().addData("Distance", trajectories.distanceToTarget(drivetrain, 14, 15));
        telemetryControl.getTelemetry().addData("vel", velocity);
        driver.readButtons();
        drivetrain.periodic();
        Scheduler.execute();
        telemetryControl.update();
    }
}
