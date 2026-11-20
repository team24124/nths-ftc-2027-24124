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
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(81, 133, 90); // inch off edge of third tile
    private final Pose point1 = poseFactory.of(81, 131, 90);

    private double x = 129;
    private final Pose point2 = poseFactory.of(x, 98.7, 0);
    private final Pose point2Control1 = poseFactory.of(113, 94, 0);
    private final Pose point2Segment1Start = poseFactory.of(x, 98.7, 90);
    private final Pose point2Segment1End = poseFactory.of(x, 98.7, 0);
    private final Pose point2Segment2Start = poseFactory.of(x, 98.7, 0);
    private final Pose point2Segment2End = poseFactory.of(x, 98.7, 0);

    private final Pose point2_5 = poseFactory.of(x - 5, 98.7, 20);

    private final Pose point3 = poseFactory.of(82, 15, -90);
    private final Pose point3Control1 = poseFactory.of(113, 30, 0);
    private final Pose point3Segment1Start = poseFactory.of(82, 15, 0);
    private final Pose point3Segment1End = poseFactory.of(82, 15, 270);

    private final Pose point4 = poseFactory.of(97, 13.4, 270);
    private final Pose point4Control1 = poseFactory.of(91, 34, 0);

    private final Pose point5 = poseFactory.of(82, 15, 270);

    private final Pose point6 = poseFactory.of(127, 46, 270);

    public Path walk() {
        return Paths.line(start, point1).constant(point1);
    }

    public Path intake1() {
        return Paths.curve(point1, point2Control1, point2).heading(Interpolator.piecewise().until(0.8, Interpolator.linear(point2Segment1Start, point2Segment1End)).until(1, Interpolator.linear(point2Segment2Start, point2Segment2End)));
    }

    public Path walk2() {
        return Paths.line(point2, point2_5).heading(Interpolator.linear(point2, point2_5));
    }

    public Path moveToShoot1() {
        return Paths.curve(point2_5, point3Control1, point3).heading(Interpolator.piecewise().until(1, Interpolator.linear(point3Segment1Start, point3Segment1End)));
    }

    public Path intake2() {
        return Paths.curve(point3, point4Control1, point4).constant(point4);
    }

    public Path moveToShoot2() {
        return Paths.line(point4, point5).constant(point5);
    }

    public Path park() {
        return Paths.line(point5, point6).constant(point6);
    }

    private Command autoSequence() {
        return race(
                robot.flywheel.autonPeriodic(),

                sequential(
                        robot.intake.extendWedge(false),
                        robot.flywheel.runFlywheel(),
                        follow(robot.drivetrain.getDrivetrain(), walk()),

                        // shoot first volley
                        waitMs(3000),
                        parallel(robot.flywheel.openGate(), robot.intake.runIntake()),
                        waitMs(500),
                        parallel(robot.flywheel.closeGate(), robot.flywheel.stopFlywheel()),

                        robot.intake.extendWedge(true),
                        follow(robot.drivetrain.getDrivetrain(), intake1()),
                        waitMs(2000),
                        follow(robot.drivetrain.getDrivetrain(), walk2()),
                        robot.flywheel.runFlywheel(),

                        // shoot second volley
                        follow(robot.drivetrain.getDrivetrain(), moveToShoot1()),
                        robot.flywheel.openGate(),
                        waitMs(700),
                        robot.flywheel.closeGate(),

                        follow(robot.drivetrain.getDrivetrain(), intake2()),
                        waitMs(2000),

                        // shoot last volley
                        follow(robot.drivetrain.getDrivetrain(), moveToShoot2()),
                        robot.intake.extendWedge(false),
                        robot.flywheel.openGate(),
                        waitMs(700),
                        parallel(robot.intake.stopIntake(), robot.flywheel.closeGate(), robot.flywheel.stopFlywheel()),

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
        PoseStorage.currentPose = new Pose(robot.drivetrain.getDrivetrain().pose().x()-70.75, robot.drivetrain.getDrivetrain().pose().y()-70.75, robot.drivetrain.getDrivetrain().pose().heading());
    }
}