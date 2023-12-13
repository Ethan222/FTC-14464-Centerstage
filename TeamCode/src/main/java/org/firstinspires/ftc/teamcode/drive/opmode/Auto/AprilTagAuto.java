/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.opmode.Auto.OpenCv.Location;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
@Disabled
@Autonomous(group = "auto")
public class AprilTagAuto extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    Location teamPropLocation;
    int correspondingAprilTagID;
    AprilTagDetection tagOfInterest;

    AprilTagIDs ids = new AprilTagIDs();
    Backdrop backdrop = ids.blueBackdrop; // blue side

    @Override
    public void runOpMode()
    {
        telemetry.addLine("Initializing...");
        telemetry.update();
        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        */
        telemetry.setMsTransmissionInterval(50);

        Robot robot = new Robot(hardwareMap);

        telemetry.addLine("robot initialized");
        telemetry.update();

        // trajectories
        Pose2d startPose = new Pose2d(11, 62.5, -Math.PI / 2);
        Vector2d[] spikeMarkPoses = {
                new Vector2d(22.5, 42),
                new Vector2d(12, 38),
                new Vector2d(.5, 42)
        };
        double forwardDistance = 26;
        double waitTime = 1.5;
        double[] backdropYs = {41, 35, 28};

        Vector2d parkCoords = new Vector2d(60, 62);
        int multiplier = -1;

        robot.drive.setPoseEstimate(startPose);

        // figure out where team prop is
        teamPropLocation = Location.CENTER;
        //correspondingAprilTagID = backdrop.center;
        int location = 0;

        double armFlipStartTime = .5, armFlipTime = 3.5;
        Vector2d backdropCoords = new Vector2d(44, backdropYs[location]);
        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    robot.gripper.gripFully(); // grip the preloaded pixels
                })
                .waitSeconds(.5)
                .lineTo(spikeMarkPoses[location]) // move to spike mark
                .turn(Math.PI)
                // place pixel on spike mark
                .addTemporalMarker(armFlipStartTime, () -> {
                    robot.armFlipper.flip();
                })
                .waitSeconds(3)
                .addTemporalMarker(armFlipStartTime + armFlipTime, () -> {
                    robot.armFlipper.stop();
                    robot.gripper.ungripFully();
                })
                //.turn(multiplier * Math.PI / 2)
                //.lineTo(backdropCoords)
                //.waitSeconds(waitTime)
                //.back(4)
                //.strafeRight(multiplier * 23)
                //.lineTo(parkCoords) // park in backstage
                .build();

        telemetry.addLine("Initialized");
        telemetry.addLine("Team prop location: " + teamPropLocation);
        telemetry.update();

        waitForStart();

        // move to corresponding spike mark
        robot.drive.followTrajectorySequence(traj);
        // place purple pixel on it
        // move so can see backdrop
        // find april tag corresponding to team prop location
        /*
        Trajectory strafe = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(1)
                .build();
        while (tagOfInterest == null && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == correspondingAprilTagID) {
                        tagOfInterest = tag;
                        break;
                    }
                }
                if(tagOfInterest == null) {
                    telemetry.addLine("can see tags but not the right one");
                }
            } else {
                telemetry.addLine("can't see any tags");
            }

            telemetry.update();
            drive.followTrajectory(strafe);
            sleep(20);
        }

        telemetry.addLine("Found april tag");
        tagToTelemetry(tagOfInterest);
        telemetry.update();
        */
        // move to backdrop
        // place yellow pixel on backdrop
        // park in backstage
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
}