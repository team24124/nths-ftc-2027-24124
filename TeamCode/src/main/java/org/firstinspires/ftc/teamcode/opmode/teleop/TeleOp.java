package org.firstinspires.ftc.teamcode.opmode.teleop;

import static com.pedropathing.ivy.groups.Groups.parallel;

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
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        // --------- DRIVER INPUTS ---------

        double x = driver.getLeftX();
        double y = -driver.getLeftY();
        if (PoseStorage.currentAlliance == PoseStorage.Alliance.RED) {
            x = -driver.getLeftX();
            y = driver.getLeftY();
        }
        double rx = -driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            robot.drivetrain.toggleSpeeds();
        }

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            alignToAT = !alignToAT;
        }

        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            Scheduler.schedule(robot.intake.runIntake());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            Scheduler.schedule(robot.intake.reverseIntake());
        }
        if (driver.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER) || driver.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            Scheduler.schedule(robot.intake.stopIntake());
        }

        // --------- OPERATOR INPUTS ---------

        if (operator.wasJustPressed(GamepadKeys.Button.B)) {
            Scheduler.schedule(robot.flywheel.runFlywheel());
            operator.gamepad.rumble(200);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.A)) {
            Scheduler.schedule(robot.flywheel.stopFlywheel());
            operator.gamepad.rumble(200);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            Scheduler.schedule(parallel(robot.flywheel.openGate()), robot.intake.runIntake());
        }
        if (operator.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            Scheduler.schedule(parallel(robot.flywheel.closeGate()), robot.intake.stopIntake());
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
                robot.drivetrain.drive(x, y, trajectories.theta(robot.drivetrain, targetX, targetY), true);
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