package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public enum TeleOpTrajectories {
    INSTANCE;

    // Aligns by strafing
    public Action vectorAlign(Drivetrain drivetrain, Vector2d targetPose) {
        return drivetrain.getDrivetrain().follow()
    }

    // Aligns by strafing and turning
    public Action poseAlign(MecanumDrive drivetrain, Pose2d targetPose) {
        return drivetrain.actionBuilder(drivetrain.localizer.getPose(), true)
                .strafeToSplineHeading(new Vector2d(targetPose.position.x, targetPose.position.y), targetPose.heading.toDouble()) // 0 to face directly into target
                .build();
    }

    // Returns angle to target [-pi (left facing error), pi (right facing error)]
    public double theta(Drivetrain drivetrain, double targetX, double targetY) {
        double heading = (drivetrain.getHeading() + Math.PI/2) % (Math.PI*2);
        double botX = drivetrain.getPosition().x();
        double botY = drivetrain.getPosition().y();

        double theta = Math.atan2(targetX - botX, -targetY + botY); // Similar to (y, x) -> x is vertical, y is lateral +left (reversed to accommodate atan2)
        if (theta < 0) theta += Math.PI*2;

        if ((heading - theta) > Math.PI) {
            return -Math.PI + ((heading - theta) % Math.PI);
        }
        if (Math.abs(heading - theta) > Math.PI) {
            return Math.PI + ((heading - theta) % Math.PI);
        }

        return heading - theta;
    }

    // Returns distance to target in inches
    public double distanceToTarget(Drivetrain drivetrain, double targetX, double targetY) {
        double botX = drivetrain.getPosition().position.x;
        double botY = drivetrain.getPosition().position.y;

        botX = Math.abs(botX - targetX);
        botY = Math.abs(botY - targetY);

        return Math.hypot(botX, botY);
    }
}
