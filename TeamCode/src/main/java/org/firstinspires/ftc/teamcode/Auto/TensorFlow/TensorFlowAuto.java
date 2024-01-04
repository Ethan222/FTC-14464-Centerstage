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

package org.firstinspires.ftc.teamcode.Auto.TensorFlow;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Auto.Alliance;
import org.firstinspires.ftc.teamcode.Auto.Location;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;
import java.util.concurrent.TimeUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "TensorFlow Auto", group = "auto", preselectTeleOp = "TeleOp")
public class TensorFlowAuto extends LinearOpMode
{
    private Alliance alliance = Alliance.RED;

    private enum Side {
        BACK, FRONT
    }
    private Side side = Side.BACK;

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
        ExposureControl exposureControl;
        int exposure = 25;

        // init loop - select alliance and side
        while(!opModeIsActive() && !isStopRequested()) {
            if((gamepad1.x) || gamepad2.x)
                alliance = Alliance.BLUE;
            else if((gamepad1.b && !gamepad1.start) || (gamepad2.b && !gamepad2.start))
                alliance = Alliance.RED;
            else if(gamepad1.y || gamepad2.y)
                side = Side.BACK;
            else if((gamepad1.a && !gamepad1.start) || (gamepad2.a && !gamepad2.start))
                side = Side.FRONT;
            else if(gamepad1.dpad_left || gamepad2.dpad_left)
                propLocation = Location.LEFT;
            else if(gamepad1.dpad_up || gamepad2.dpad_up)
                propLocation = Location.CENTER;
            else if(gamepad1.dpad_right || gamepad2.dpad_right)
                propLocation = Location.RIGHT;

            try {
                exposureControl = propDetector.visionPortal.getCameraControl(ExposureControl.class);
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
                telemetry.addData("exposure", exposureControl.getExposure(TimeUnit.MILLISECONDS));
                if(gamepad1.right_trigger > 0)
                    exposure++;
                else if(gamepad1.left_trigger > 0)
                    exposure--;
            } catch(Exception e) {
                telemetry.addLine("camera isn't initialized yet");
            }

            propLocation = propDetector.getLocation();

            telemetry.addLine("\nInitialized");
            telemetry.addLine(String.format("Alliance: %s (x = blue, b = red)", alliance));
            telemetry.addLine(String.format("Side: %s (a = front, y = back)", side));
            telemetry.addLine();
            telemetry.addData("Team prop location", propLocation);
            telemetry.addLine();
            propDetector.telemetryAll(telemetry);
            telemetry.update();
        }
        // start of op mode
        //propDetector.stopDetecting();
        telemetry.setMsTransmissionInterval(250);
        ElapsedTime time = new ElapsedTime();

        // trajectories
        Pose2d startPose, parkPose;
        TrajectorySequence spikeMarkTraj = null;
        TrajectorySequence parkTraj;
        int multiplier = side == Side.FRONT ? 1 : -1;

        if(alliance == Alliance.BLUE && side == Side.BACK) {
            // BLUE BACK (left)
            startPose = new Pose2d(12, 62.5, -Math.PI / 2);
            parkPose = propLocation == Location.CENTER ? new Pose2d(61, 59, Math.PI) : new Pose2d(61, 58, Math.PI);
            switch(propLocation) {
                case LEFT:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(22, 28, -Math.PI/2), -Math.PI/2)
                            .build();
                    break;
                case CENTER:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(13, 20.5, Math.PI), Math.PI)
                            .build();
                    break;
                case RIGHT:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(20, 39, -3*Math.PI/4), -3*Math.PI/4)
                            .splineToLinearHeading(new Pose2d(1, 32, Math.PI), Math.PI)
                            .build();
            }
        }
        else if(alliance == Alliance.BLUE && side == Side.FRONT) {
            // BLUE FRONT (right)
            startPose = new Pose2d(-36, 62, -Math.PI / 2);
            parkPose = new Pose2d(60, 14, Math.PI);
            switch(propLocation) {
                case LEFT:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-20, 39, -Math.PI/4), -Math.PI/4)
                            .splineToLinearHeading(new Pose2d(-1, 32, 0), 0)
                            .build();
                    break;
                case CENTER:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-13, 20.5, 0), 0)
                            .build();
                    break;
                case RIGHT:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-22, 28, -Math.PI/2), -Math.PI/2)
                            .build();
            }
        }
        else if(alliance == Alliance.RED && side == Side.BACK) {
            // RED BACK (right)
            startPose = new Pose2d(12, -61, Math.PI / 2);
            parkPose = new Pose2d(60, -59, Math.PI);
            switch(propLocation) {
                case LEFT:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(16, -47, 3*Math.PI/4), 3*Math.PI/4)
                            .splineToLinearHeading(new Pose2d(0, -30, Math.PI), Math.PI)
                            .build();
                    break;
                case CENTER:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(12, -20.5, Math.PI), Math.PI)
                            .build();
                    break;
                case RIGHT:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(22.5, -26, Math.PI/2), Math.PI/2)
                            .build();
            }
        }
        else {
            // RED FRONT (left)
            startPose = new Pose2d(-36, -61, Math.PI / 2);
            parkPose = new Pose2d(60, -11, Math.PI);
        }

        double intakeWaitTime = 1;

        Trajectory[] moveForwardALittle = new Trajectory[10];
        moveForwardALittle[0] = robot.drive.trajectoryBuilder(startPose)
                .forward(2)
                .build();
        for(int i = 1; i < 10; i++) {
            moveForwardALittle[i] = robot.drive.trajectoryBuilder(moveForwardALittle[i - 1].end())
                    .forward(2)
                    .build();
        }
        TrajectorySequence placePixel = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                .waitSeconds(intakeWaitTime)
                .back(14)
                .build();
        Trajectory park = robot.drive.trajectoryBuilder(placePixel.end(), true)
                .splineToLinearHeading(parkPose, 0) // park in backstage
                .build();

        robot.drive.setPoseEstimate(startPose);
        List<Recognition> recognitions = propDetector.getRecognitions();
        int i = 0;
//        while(recognitions.size() == 0 && i < 6 && opModeIsActive() && time.seconds() < 15) {
//            robot.drive.followTrajectory(moveForwardALittle[i]);
//            recognitions = propDetector.getRecognitions();
//            telemetry.addLine("moving forward");
//            propDetector.telemetryAll(telemetry);
//            telemetry.update();
//            sleep(1000);
//            i++;
//        }
        while(opModeIsActive());
        robot.drive.followTrajectorySequence(spikeMarkTraj);
        robot.gripper.ungripFully();
        robot.intake.out();
        robot.drive.followTrajectorySequence(placePixel);
        robot.intake.stop();
        robot.drive.followTrajectory(park);
    }
}