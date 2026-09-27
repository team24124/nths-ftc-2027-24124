package org.firstinspires.ftc.teamcode.opmode.debug.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
    public static boolean robotCentric = true;
    private boolean state = true;
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
            Vector2d targetPose = new Vector2d(xPos, yPos);
            actions.schedule(trajectories.vectorAlign(drivetrain.getDrive(), targetPose));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            Pose2d targetPose = new Pose2d(xPos, yPos, heading);
            actions.schedule(trajectories.poseAlign(drivetrain.getDrive(), targetPose));
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

        if (driver.wasJustPressed(GamepadKeys.Button.B) && state != robotCentric) {
            switchDrive();
            state = robotCentric;
        }

        if (!drivetrain.getDrive().isBusy) {
            drivetrain.drive(x, y, rx, false);
        }
        drivetrain.periodic();

        driver.readButtons();

        actions.run();

        telemetry.addData("\nBusy", drivetrain.getDrive().isBusy);
        telemetry.addData("\nX", "%.1f", drivetrain.getPosition().component1().x);
        telemetry.addData("Y", "%.1f", drivetrain.getPosition().component1().y);
        telemetry.addData("\nHeading", "%.1f", drivetrain.getPosition().heading.toDouble());
        telemetry.addData("\nStored Pose", PoseStorage.currentPose.toString());

        telemetry.addData("\n\nTargeted X", xPos);
        telemetry.addData("Targeted Y", yPos);
        telemetry.addData("Targeted Heading", heading);

        telemetry.addData("\n\nDistance to (0, 0)", trajectories.distanceToTarget(drivetrain, 0, 0));
        telemetry.addData("\nAngle to (0, 0)", Math.toDegrees(trajectories.theta(drivetrain, 0, 0)));

        telemetry.update();
    }

    @Override
    public void stop() {
        actions.stop();
    }
}