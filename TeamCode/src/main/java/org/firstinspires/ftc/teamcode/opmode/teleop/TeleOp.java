package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.ivy.Scheduler;
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
    private boolean notified = false;
    private final double targetX = PoseStorage.currentAlliance == PoseStorage.Alliance.BLUE ? 14 : -14;
    private double targetY = 14;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        trajectories = TeleOpTrajectories.INSTANCE;
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new Robot(hardwareMap, telemetry);

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

        // --------- DRIVER INPUTS ---------

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

        if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            Scheduler.schedule(robot.intake.runIntake());
        } else if (driver.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            Scheduler.schedule(robot.intake.reverseIntake());
        } else {
            Scheduler.schedule(robot.intake.stopIntake());
        }

        // --------- OPERATOR INPUTS ---------

        if (operator.wasJustPressed(GamepadKeys.Button.B)) {
            Scheduler.schedule(robot.flywheel.runFlywheel());
            driver.gamepad.rumble(200);
            notified = false;
        }

        if (operator.wasJustPressed(GamepadKeys.Button.A)) {
            Scheduler.schedule(robot.flywheel.stopFlywheel());
            driver.gamepad.rumble(200);
            notified = false;
        }

        if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (robot.flywheel.primed) {
                Scheduler.schedule();
            }
        }

        if (robot.flywheel.primed && !notified) {
            operator.gamepad.rumbleBlips(3);
            driver.gamepad.rumbleBlips(3);
            notified = true;
        }

        if (operator.wasJustPressed(GamepadKeys.Button.Y)) {
            targetY = -targetY;
        }

        // --- PERIODIC CALLS ---

        driver.readButtons();
        operator.readButtons();

        robot.flywheel.periodic();
        robot.drivetrain.periodic();

        if (!robot.drivetrain.getDrivetrain().isBusy()) {
            if (alignToAT) {
                if (robot.limelight.isDetected()) {
                    robot.drivetrain.drive(x, y, Math.toRadians(robot.limelight.degreeOffset()), true);
                } else {
                    robot.drivetrain.drive(x, y, trajectories.theta(robot.drivetrain, targetX, targetY), true);
                }
            } else {
                robot.drivetrain.drive(x, y, rx, false);
            }
        }

        double d = trajectories.distanceToTarget(robot.drivetrain, targetX, targetY);
        Scheduler.schedule(robot.flywheel.setVls(d));

        Scheduler.execute();
    }

    @Override
    public void stop() {
        robot.telemetryControl.unsubscribeAll();
    }
}