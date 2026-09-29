package org.firstinspires.ftc.teamcode.opmode.auton.supporters;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.through;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "BLUE idle")
public class BLUEidle extends OpMode {
    Robot robot;
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.radians();
    private final Pose startPose = poseFactory.of(24, 24, 0);
    private final Pose scorePose = poseFactory.of(48, 48, Math.toRadians(90));
    private final Pose parkPose = poseFactory.of(72, 48, Math.toRadians(90));

    private Path startToScore() {
        return through(startPose, scorePose, parkPose).linear(startPose, parkPose);
    }

    private Path park(){
        return line(scorePose, parkPose).linear(scorePose, parkPose);
    }

    private Command auto() {
        return sequential(
                follow(follower, startToScore()),
                // Add mechanism commands here.
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        PoseStorage.currentPose = startPose;
        PoseStorage.currentAlliance = PoseStorage.Alliance.BLUE;
    }

    @Override
    public void start() {
        schedule(auto());
        PoseStorage.currentPose = follower.pose();
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}