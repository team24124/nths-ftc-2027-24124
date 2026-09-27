package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "!")
public class TeleOp extends OpMode {
    private List<LynxModule> hubs;
    private Robot robot;
    private GamepadEx driver, operator;
    private TeleOpTrajectories trajectories;
    private boolean alignToAT = false;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        trajectories = TeleOpTrajectories.INSTANCE;
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap, telemetry, false);
        robot.actions.init();

        if (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) {
            robot.limelight.setPipeline(Limelight.Pipeline.AT3);
        } else {
            robot.limelight.setPipeline(Limelight.Pipeline.AT2);
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        // --------- Driver inputs ---------
        double x = -driver.getLeftY();
        double y = driver.getLeftX();
        if (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) {
            x = driver.getLeftY();
            y = -driver.getLeftX();
        }
        double rx = driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            robot.drivetrain.toggleSpeeds();
        }

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            alignToAT = !alignToAT;
        }

        // USE THESE
        driver.gamepad.rumbleBlips(3);
        driver.gamepad.rumble(1.0, 1.0, 3);

        boolean align0 = driver.isDown(GamepadKeys.Button.B);
        if (align0) {
            alignToAT = false;
            robot.drivetrain.drive(x, y, robot.drivetrain.getHeading(), true);
        }

        if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {

        }

        // --------- Operator inputs ---------
        if (operator.wasJustPressed(GamepadKeys.Button.B)) {

        }

        if (operator.wasJustPressed(GamepadKeys.Button.A)) {

        }

        if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {

        }

        // --- Periodic calls ---
        driver.readButtons();
        operator.readButtons();

        robot.drivetrain.periodic();
        if (!robot.drivetrain.getDrivetrain().isBusy()) {
            if (alignToAT) {
                if (robot.limelight.isDetected()) {
                    robot.drivetrain.drive(x, y, Math.toRadians(robot.limelight.degreeOffset()), true);
                } else {
                    if (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) {
                        robot.drivetrain.drive(x, y, trajectories.theta(robot.drivetrain, 72, -72), true);
                    } else {
                        robot.drivetrain.drive(x, y, trajectories.theta(robot.drivetrain, 72, 72), true);
                    }
                }
            } else if (!align0){
                robot.drivetrain.drive(x, y, rx, false);
            }
        }

        double d;
        if (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) {
            d = trajectories.distanceToTarget(robot.drivetrain, 72, -72);
        } else {
            d = trajectories.distanceToTarget(robot.drivetrain, 72, 72);
        }

        robot.actions.run();
    }

    @Override
    public void stop() {
        robot.telemetryControl.unsubscribeAll();
        robot.actions.stop();
    }
}