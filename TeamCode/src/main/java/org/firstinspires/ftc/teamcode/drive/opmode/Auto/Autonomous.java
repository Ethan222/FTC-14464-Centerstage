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
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.opmode.Auto.OpenCv.Alliance;
import org.firstinspires.ftc.teamcode.drive.opmode.Auto.OpenCv.Location;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "auto", preselectTeleOp = "TeleOp")
public class Autonomous extends LinearOpMode
{
    private Alliance alliance = Alliance.BLUE;

    private enum Side {
        LEFT, RIGHT
    }
    private Side side = Side.LEFT;

    Robot robot;
    TensorFlowObjectDetector propDetector;

    private Location propLocation = Location.CENTER;

    @Override
    public void runOpMode()
    {
        telemetry.addLine("Initializing...");
        telemetry.update();

        telemetry.setMsTransmissionInterval(50); // default is 250

        robot = new Robot(hardwareMap);
        propDetector = new TensorFlowObjectDetector(hardwareMap);

        // init loop - select alliance and side
        while(!opModeIsActive() && !isStopRequested()) {
            if((gamepad1.x) || gamepad2.x)
                alliance = Alliance.BLUE;
            else if((gamepad1.b && !gamepad1.start) || (gamepad2.b && !gamepad2.start))
                alliance = Alliance.RED;
            else if(gamepad1.left_bumper || gamepad2.left_bumper)
                side = Side.LEFT;
            else if(gamepad1.right_bumper || gamepad2.right_bumper)
                side = Side.RIGHT;
            else if(gamepad1.dpad_left || gamepad2.dpad_left)
                propLocation = Location.LEFT;
            else if(gamepad1.dpad_up || gamepad2.dpad_up)
                propLocation = Location.CENTER;
            else if(gamepad1.dpad_right || gamepad2.dpad_right)
                propLocation = Location.RIGHT;

            propLocation = propDetector.getLocation();

            telemetry.addLine("Initialized");
            telemetry.addLine(String.format("Alliance: %s (x = blue, b = red)", alliance));
            telemetry.addLine(String.format("Side: %s (left/right bumper)", side));
            telemetry.addLine();
            telemetry.addData("Team prop location", propLocation);
            telemetry.addLine();
            propDetector.telemetryBest(telemetry);
            telemetry.update();
        }
        // start of op mode
        propDetector.stopDetecting();
        telemetry.setMsTransmissionInterval(250);

        // trajectories
        Pose2d startPose, parkPose;
        Pose2d[] spikeMarkPoses;
        Trajectory spikeMarkTraj;
        int multiplier = side == Side.RIGHT ? 1 : -1;

        if(alliance == Alliance.BLUE && side == Side.LEFT) {
            // BLUE LEFT (backdrop)
            startPose = new Pose2d(15, 62.5, -Math.PI / 2);
            spikeMarkPoses = new Pose2d[]{
                    new Pose2d(21, 28, -Math.PI / 2),
                    new Pose2d(13, 21, Math.PI),
                    new Pose2d(.5, 27, -Math.PI / 2)
            };
            parkPose = propLocation == Location.CENTER ? new Pose2d(61, 58, Math.PI) : new Pose2d(61, 57, Math.PI);
        }
        else if(alliance == Alliance.BLUE && side == Side.RIGHT) {
            // BLUE RIGHT
            startPose = new Pose2d(-36, 62, -Math.PI / 2);
            spikeMarkPoses = new Pose2d[]{};
            parkPose = new Pose2d(60, 14, Math.PI);
        }
        else if(alliance == Alliance.RED && side == Side.LEFT) {
            // RED LEFT
            startPose = new Pose2d(-36, -61, Math.PI / 2);
            spikeMarkPoses = new Pose2d[]{};
            parkPose = new Pose2d(60, -11, Math.PI);
        }
        else {
            // RED RIGHT (backdrop)
            startPose = new Pose2d(11, -61, Math.PI / 2);
            spikeMarkPoses = new Pose2d[]{};
            parkPose = new Pose2d(60, -60, Math.PI);
        }

        if(propLocation == Location.LEFT) {
            spikeMarkTraj = robot.drive.trajectoryBuilder(startPose)
                    .splineToLinearHeading(spikeMarkPoses[0], -Math.PI / 2)
                    .build();
        } else if(propLocation == Location.CENTER) {
            spikeMarkTraj = robot.drive.trajectoryBuilder(startPose)
                    .splineToLinearHeading(spikeMarkPoses[1], Math.PI)
                    .build();
        } else {
            spikeMarkTraj = robot.drive.trajectoryBuilder(startPose)
                    .splineToLinearHeading(spikeMarkPoses[2], -Math.PI / 2)
                    .build();
        }
        double intakeWaitTime = 1;

        TrajectorySequence placePixel = robot.drive.trajectorySequenceBuilder(startPose)
                // move to spike mark
                .addTrajectory(spikeMarkTraj)
                // place pixel on spike mark
                .addTemporalMarker(() -> {
                    robot.gripper.ungripFully();
                    robot.intake.out();
                })
                .waitSeconds(intakeWaitTime)
                .back(16)
                .addTemporalMarker(() -> {
                    robot.intake.stop();
                })
                .build();
        Trajectory park = robot.drive.trajectoryBuilder(placePixel.end(), true)
                .splineToLinearHeading(parkPose, 0) // park in backstage
                .build();

        robot.drive.setPoseEstimate(startPose);
        robot.drive.followTrajectorySequence(placePixel);
        robot.drive.followTrajectory(park);
    }
}