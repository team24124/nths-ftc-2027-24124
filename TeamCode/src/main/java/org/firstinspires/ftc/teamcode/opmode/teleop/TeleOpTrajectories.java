package org.firstinspires.ftc.teamcode.opmode.teleop;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Drivetrain;

public enum TeleOpTrajectories {
    INSTANCE;

    // Aligns by strafing
    public Command vectorAlign(Drivetrain drivetrain, Pose targetPose) {
        Pose start = drivetrain.getPosition();
        return follow(drivetrain.getDrivetrain(), line(start, targetPose).linear(start, start));
    }

    // Aligns by strafing and turning
    public Command poseAlign(Drivetrain drivetrain, Pose targetPose) {
        Pose start = drivetrain.getPosition();
        return follow(drivetrain.getDrivetrain(), line(start, targetPose).linear(start, targetPose));
    }

    // Returns angle to target [-pi (left facing error), +pi (right facing error)]
    public double theta(Drivetrain drivetrain, double targetX, double targetY) {
        double heading = drivetrain.getPosition().heading();
        double botX = drivetrain.getPosition().x();
        double botY = drivetrain.getPosition().y();

        double target = Math.atan2(targetY - botY, targetX - botX);

        double diff = heading - target;

        return Math.atan2(Math.sin(diff), Math.cos(diff));
    }


    // Returns distance to target in inches
    public double distanceToTarget(Drivetrain drivetrain, double targetX, double targetY) {
        double dx = targetX - drivetrain.getPosition().x();
        double dy = targetY - drivetrain.getPosition().y();

        return Math.hypot(dx, dy);
    }
}
