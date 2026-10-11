package org.firstinspires.ftc.teamcode.opmode.auton;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "BLUE main")
public class BLUEmain extends OpMode {
    Robot robot;
    private final PoseFactory poseFactory = PoseFactory.radians();
    private final Pose start = poseFactory.of(85, 136, 90);

    private final Pose point1 = poseFactory.of(133, 133, 0);
    private final Pose point1Control1 = poseFactory.of(85, 123, 0);
    private final Pose point1Control2 = poseFactory.of(102, 136, 0);
    private final Pose point1Segment1Start = poseFactory.of(133, 133, 90);
    private final Pose point1Segment1End = poseFactory.of(133, 133, 0);
    private final Pose point1Segment2Start = poseFactory.of(133, 133, 0);
    private final Pose point1Segment2End = poseFactory.of(133, 133, 0);

    private final Pose point2 = poseFactory.of(84, 17, -90);
    private final Pose point2Control1 = poseFactory.of(113, 23, 0);
    private final Pose point2Segment1Start = poseFactory.of(84, 17, 0);
    private final Pose point2Segment1End = poseFactory.of(84, 17, 75);
    private final Pose point2Segment2Start = poseFactory.of(84, 17, 75);
    private final Pose point2Segment2End = poseFactory.of(84, 17, 70);
    private final Pose point2Segment3Start = poseFactory.of(84, 17, 70);
    private final Pose point2Segment3End = poseFactory.of(84, 17, 270);

    private final Pose point3Start = poseFactory.of(84, 17, 270);
    private final Pose point3 = poseFactory.of(133, 39, 0);
    private final Pose point3Control1 = poseFactory.of(89, 38, 0);

    private final Pose point4 = poseFactory.of(84, 14, 270);
    private final Pose point4Control1 = poseFactory.of(86, 38, 0);

    private final Pose point5 = poseFactory.of(130, 50, 0);

    public Path intake() {
        return Paths.curve(start, point1Control1, point1Control2, point1).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(point1Segment1Start, point1Segment1End)).until(1, Interpolator.linear(point1Segment2Start, point1Segment2End)));
    }
    public Path moveToShoot1() {
        return Paths.curve(point1, point2Control1, point2).heading(Interpolator.piecewise().until(0.1, Interpolator.linear(point2Segment1Start, point2Segment1End)).until(0.6, Interpolator.linear(point2Segment2Start, point2Segment2End)).until(1, Interpolator.linear(point2Segment3Start, point2Segment3End)));
    }
    public Path intake2() {
        return Paths.curve(point3Start, point3Control1, point3).linear(point3Start, point3);
    }
    public Path moveToShoot2() {
        return Paths.curve(point3, point4Control1, point4).linear(point3, point4);
    }
    public Path park() {
        return Paths.line(point4, point5).linear(point4, point5);
    }

    private Command autoSequence() {
        return race(
                robot.flywheel.autonPeriodic(),

                sequential(
                        // shoot first volley
                        robot.flywheel.runFlywheel(),
                        waitMs(1000),
                        parallel(robot.flywheel.openGate(), robot.intake.runIntake()),
                        waitMs(500),
                        parallel(robot.flywheel.closeGate(), robot.flywheel.stopFlywheel()),

                        follow(robot.drivetrain.getDrivetrain(), intake()),
                        waitMs(300),
                        robot.flywheel.runFlywheel(),

                        // shoot second volley
                        follow(robot.drivetrain.getDrivetrain(), moveToShoot1()),
                        robot.flywheel.openGate(),
                        waitMs(500),
                        parallel(robot.flywheel.closeGate(), robot.flywheel.stopFlywheel()),

                        follow(robot.drivetrain.getDrivetrain(), intake2()),
                        waitMs(300),
                        robot.flywheel.runFlywheel(),

                        // shoot last volley
                        follow(robot.drivetrain.getDrivetrain(), moveToShoot2()),
                        robot.flywheel.openGate(),
                        waitMs(500),
                        parallel(robot.flywheel.closeGate(), robot.intake.stopIntake(), robot.flywheel.stopFlywheel()),

                        follow(robot.drivetrain.getDrivetrain(), park())
                )
        );
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.getDrivetrain().setPose(start);
        robot.drivetrain.getDrivetrain().update();

        Scheduler.reset();

        PoseStorage.currentPose = start;
        PoseStorage.currentAlliance = PoseStorage.Alliance.BLUE;
    }

    @Override
    public void start() {
        schedule(autoSequence());
    }

    @Override
    public void loop() {
        robot.drivetrain.getDrivetrain().update();
        Scheduler.execute();

        telemetry.addData("X", robot.drivetrain.getDrivetrain().pose().x());
        telemetry.addData("Y", robot.drivetrain.getDrivetrain().pose().y());
        telemetry.addData("Heading", Math.toDegrees(robot.drivetrain.getDrivetrain().pose().heading()));
        telemetry.addData("Follower Mode", robot.drivetrain.getDrivetrain().mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = robot.drivetrain.getDrivetrain().pose();
    }
}