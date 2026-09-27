/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.opmode.debug.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;

import java.util.List;

/**
 *   @see <a href="https://limelightvision.io/">Limelight</a><br>
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.<br>
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".<br>
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */

@Config
@TeleOp(name = "Limelight", group = "test")
public class LimelightDebugger extends LinearOpMode {
    private Limelight limelight;
    private List<LynxModule> hubs;

    @Override
    public void runOpMode() throws InterruptedException {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        limelight = new Limelight(hardwareMap);
        limelight.setPipeline(Limelight.Pipeline.AT1);

        telemetry.setMsTransmissionInterval(11);

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            limelight.setPipeline(Limelight.Pipeline.AT1);

            LLStatus status = limelight.status();
            telemetry.addData("\nName", "%s", status.getName());
            telemetry.addData("\nLL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(), (int)status.getFps());
            telemetry.addData("\nPipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getResult();
            if (limelight.isDetected()) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("\nLL Latency", captureLatency + targetingLatency);
                telemetry.addData("\nParse Latency", parseLatency);
                telemetry.addData("\nPythonOutput", java.util.Arrays.toString(result.getPythonOutput()));


                telemetry.addData("\ntxnc", result.getTxNC());
                telemetry.addData("tync", result.getTyNC());

                telemetry.addData("\nbotPose", botpose.toString());
                telemetry.addData("botPose", limelight.ATRobotPoseFieldSpace().toString());

                // Access fiducial results (same format accessing for all other results if needed)
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("\nFiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                if (!fiducialResults.isEmpty()) {
                    telemetry.addData("\nDistance", limelight.distance());
                    telemetry.addData("Degree offset", limelight.degreeOffset());
                    telemetry.addData("\nTarget Vector", limelight.targetVectorFieldSpace(new Pose2d(0, 0, 0)).toString());
                    telemetry.addData("\nTarget Vector AT", limelight.ATTargetVectorFieldSpace(new Pose2d(0, 0, 0)).toString());
                    telemetry.addData("\nTarget Pose AT", limelight.ATTargetPoseFieldSpace(new Pose2d(0, 0, 0)).toString());
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }
    }
}
