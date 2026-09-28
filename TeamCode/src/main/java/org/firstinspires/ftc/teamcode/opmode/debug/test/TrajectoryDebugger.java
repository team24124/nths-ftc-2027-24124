package org.firstinspires.ftc.teamcode.opmode.debug.test;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.List;

@Config
@TeleOp(name = "Trajectories", group = "test")
public class TrajectoryDebugger extends OpMode {
    private Drivetrain drivetrain;
    private GamepadEx driver;
    private TeleOpTrajectories trajectories;
    private List<LynxModule> hubs;
    private double xPos = 0;
    private double yPos = 0;
    private double heading = 0;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drivetrain = new Drivetrain(hardwareMap, new Pose(0, 0, 0));
        trajectories = TeleOpTrajectories.INSTANCE;
        driver = new GamepadEx(gamepad1);
        Scheduler.reset();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        double y = driver.getLeftY();
        double x = driver.getLeftX();
        double rx = driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            Pose targetPose = new Pose(xPos, yPos, heading);
            schedule(trajectories.vectorAlign(drivetrain, targetPose));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            Pose targetPose = new Pose(xPos, yPos, heading);
            schedule(trajectories.poseAlign(drivetrain, targetPose));
        }

        // Adjust x and y
        if (driver.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            yPos -= 0.08;
        } else if (driver.isDown(GamepadKeys.Button.DPAD_LEFT)) {
            yPos += 0.08;
        }
        if (driver.isDown(GamepadKeys.Button.DPAD_UP)) {
            xPos += 0.08;
        } else if (driver.isDown(GamepadKeys.Button.DPAD_DOWN)) {
            xPos -= 0.08;
        }
        if (driver.isDown(GamepadKeys.Button.Y)) {
            heading += 0.04;
        } else if (driver.isDown(GamepadKeys.Button.A)) {
            heading -= 0.04;
        }

        if (!drivetrain.getDrivetrain().isBusy()) {
            drivetrain.drive(x, y, rx, false);
        }
        drivetrain.periodic();

        driver.readButtons();

        Scheduler.execute();

        telemetry.addData("\nBusy", drivetrain.getDrivetrain().isBusy());
        telemetry.addData("\nX", "%.1f", drivetrain.getPosition().x());
        telemetry.addData("Y", "%.1f", drivetrain.getPosition().y());
        telemetry.addData("\nHeading", "%.1f", drivetrain.getPosition().heading());
        telemetry.addData("\nStored Pose", PoseStorage.currentPose.toString());

        telemetry.addData("\n\nTargeted X", xPos);
        telemetry.addData("Targeted Y", yPos);
        telemetry.addData("Targeted Heading", heading);

        telemetry.addData("\n\nDistance to (0, 0)", trajectories.distanceToTarget(drivetrain, 0, 0));
        telemetry.addData("\nAngle to (0, 0)", Math.toDegrees(trajectories.theta(drivetrain, 0, 0)));

        telemetry.update();
    }
}