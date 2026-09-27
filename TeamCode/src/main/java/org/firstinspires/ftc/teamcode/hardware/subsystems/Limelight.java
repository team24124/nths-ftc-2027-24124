package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Limelight implements SubsystemBase, TelemetryObservable {
    private final Limelight3A limelight;
    public enum Pipeline {
        PSS1(0), // Python snapscript
        AT1(1), // Empty
        AT2(2), // Full 3D BLUE
        AT3(3), // Full 3D RED
        AT4(4); // MT2 for pose reset

        public final int pipelineNum;

        Pipeline(int pipelineNum) {
            this.pipelineNum = pipelineNum;
        }
    }

    private Pipeline pipeline;

    private final double cameraAngle = 0; // Facing forward == 0 degrees

    public Limelight(HardwareMap hw){
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(Pipeline.AT1.pipelineNum);
        limelight.setPollRateHz(100); // Ask Limelight for data (100 times per second)
        limelight.start(); // Starts polling for data. If start() is neglected, getLatestResult() will return null
        pipeline = Pipeline.AT1;
    }

    public LLResult getResult() { return limelight.getLatestResult(); }
    public LLStatus status() {
        return limelight.getStatus();
    }
    public void setPipeline(Pipeline pipeline) {
        if (pipeline != this.pipeline) {
            this.pipeline = pipeline;
            limelight.pipelineSwitch(pipeline.pipelineNum);
        }
    }
    public boolean isDetected() {
        return getResult().isValid();
    }
    private LLResultTypes.FiducialResult getFirstAprilTag() {
        List<LLResultTypes.FiducialResult> fiducials = getResult().getFiducialResults();
        return fiducials.get(0);
    }

    private double normalizeAngleTo2Pi(double a) {
        a = a % (2 * Math.PI);
        if (a < 0) a += 2 * Math.PI;
        return a;
    }


    //---------------------------------- values for PIDF ----------------------------------

    // Degree offset for rotation, +east -west
    public double degreeOffset() {
        return getResult().getTx();
    }

    // Distance in inches, AT height / tan of forward to angle to target
    public double distance() {
        return 999 / Math.tan(Math.toRadians(cameraAngle + getResult().getTyNC()));
    }

    //---------------------------------- target position returners ----------------------------------

    // Field space X and Y offset for detected
    public Pose2d targetVectorFieldSpace(Pose2d botPose) {
        // Target raw data
        double ty = 29.5 / Math.tan(Math.toRadians(cameraAngle + getResult().getTyNC())); // Target height (opposite) / ratio to find adjacent (robot distance)
        double tx = Math.tan(Math.toRadians(0 + getResult().getTxNC())) * ty; // +0 for limelight yaw. Multiply tan by adjacent to get opposite

        // Desired pose relative to target
        double x = tx - 0;
        double y = ty - 100;

        double heading = botPose.heading.toDouble();

        // Target converted into field coordinates
        double x1 = botPose.position.x + (x * Math.cos(heading) - y * Math.sin(heading));
        double y1 = botPose.position.y + (x * Math.sin(heading) + y * Math.cos(heading));

        return new Pose2d(x1, y1, heading);
    }

    // Field space X and Y offset for apriltag
    public Pose2d ATTargetVectorFieldSpace(Pose2d botPose) {
        Pose3D targetPose = getFirstAprilTag().getTargetPoseRobotSpace();

        // Robot space is cartesian with X+ pointing forward and Y+ pointing right
        double x = targetPose.getPosition().y * 39.37 - 0;
        double y = targetPose.getPosition().x * 39.37 - 50;

        double heading = botPose.heading.toDouble();

        // Same conversion steps as method above
        double x1 = botPose.position.x + (x * Math.cos(heading) - y * Math.sin(heading));
        double y1 = botPose.position.y + (x * Math.sin(heading) + y * Math.cos(heading));

        return new Pose2d(x1, y1, heading);
    }

    // Robot space X and Y offset and heading for apriltag
    public Pose2d ATTargetPoseFieldSpace(Pose2d botPose) {
        Pose3D pose = getFirstAprilTag().getTargetPoseRobotSpace();
        double theta = Math.toRadians(degreeOffset());

        double x = pose.getPosition().y * 39.37;
        double y = pose.getPosition().x * 39.37 - 50;

        double heading = botPose.heading.toDouble();

        double x1 = botPose.position.x + (x * Math.cos(heading) - y * Math.sin(heading));
        double y1 = botPose.position.y + (x * Math.sin(heading) + y * Math.cos(heading));

        return new Pose2d(x1, y1, normalizeAngleTo2Pi(heading - theta)); // Combine heading and rotation counter
    }

    // Field space bot pose (this is mapped, face desired AT ID 20 or 24. All other AT's are neglected due to uncertain placement)
    public Pose2d ATRobotPoseFieldSpace() {
        Pose3D pose3d = getResult().getBotpose_MT2();
        return new Pose2d(pose3d.getPosition().x * 39.37, pose3d.getPosition().y * 39.37, normalizeAngleTo2Pi(pose3d.getOrientation().getYaw(AngleUnit.RADIANS)));
    }

    //---------------------------------- telemetry ----------------------------------

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        LLResult result = getResult();
        telemetry.addData("Status", limelight.getStatus().toString());
        telemetry.addData("pipeline", pipeline.name());
        if (getResult().isValid() && result.getPythonOutput() != null) {
            double[] pythonOutputs = result.getPythonOutput();
            telemetry.addData("validity", pythonOutputs[0]);
            telemetry.addData("targetX", Math.round(pythonOutputs[1]));
            telemetry.addData("targetY", Math.round(pythonOutputs[2]));
            telemetry.addData("angle", Math.round(pythonOutputs[3]));
            telemetry.addData("area %", Math.round(pythonOutputs[4]));
        } else if (getResult().isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getTargetPoseRobotSpace(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
        }
    }

    @Override
    public String getName() {
        return "Limelight";
    }
}
