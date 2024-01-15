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
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.Auto.Alliance;
import org.firstinspires.ftc.teamcode.Auto.Location;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "TensorFlow Auto", group = "auto", preselectTeleOp = "TeleOp")
public class TensorFlowAuto extends LinearOpMode
{
    private enum Side {
        BACK, FRONT
    }

    private static Alliance alliance = Alliance.BLUE;
    private static Side side = Side.BACK;

    Robot robot;
    TensorFlowObjectDetector propDetector;

    private Location propLocation = Location.LEFT;

    @Override
    public void runOpMode()
    {
        boolean initialized = false;
        double timeToCameraInit = 0;

        robot = new Robot(hardwareMap);

        propDetector = new TensorFlowObjectDetector(hardwareMap);
        ExposureControl exposureControl = null;
        //int exposure = 50;
        //telemetry.setMsTransmissionInterval(50); // default is 250
        boolean propLocationOverride = false;

        ElapsedTime timer = new ElapsedTime();

        // init loop - select alliance and side
        while(opModeInInit()) {
            if((gamepad1.x) || gamepad2.x)
                alliance = Alliance.BLUE;
            else if((gamepad1.b && !gamepad1.start) || (gamepad2.b && !gamepad2.start))
                alliance = Alliance.RED;
            else if(gamepad1.y || gamepad2.y)
                side = Side.BACK;
            else if((gamepad1.a && !gamepad1.start) || (gamepad2.a && !gamepad2.start))
                side = Side.FRONT;
            else if(gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right || gamepad2.dpad_left || gamepad2.dpad_up || gamepad2.dpad_right)
                propLocationOverride = true;
            else if(gamepad1.back || gamepad2.back)
                propLocationOverride = false;
            else if((gamepad1.start && gamepad1.back) || (gamepad2.start && gamepad2.back))
                requestOpModeStop();

            if(propLocationOverride) {
                if(gamepad1.dpad_left || gamepad2.dpad_left)
                    propLocation = Location.LEFT;
                else if(gamepad1.dpad_up || gamepad2.dpad_up)
                    propLocation = Location.CENTER;
                else if(gamepad1.dpad_right || gamepad2.dpad_right)
                    propLocation = Location.RIGHT;
            }

            telemetry.addLine(initialized ? "Initialized" : String.format(Locale.ENGLISH, "Initializing... %.1f", 3 - getRuntime()));
            telemetry.addData("Runtime", "%.1f", getRuntime());
            telemetry.addData("Camera init time", "%.2f", timeToCameraInit);
            telemetry.addLine(String.format("\nAlliance: %s (x = blue, b = red)", alliance));
            telemetry.addLine(String.format("Side: %s (a = front, y = back)", side));
            if(propLocationOverride)
                telemetry.addData("\nProp location", "%s (override)", propLocation);
            else
                telemetry.addData("\nProp location", initialized ? propLocation : null);

            if(!initialized && !propLocationOverride) {
                try {
                    exposureControl = propDetector.visionPortal.getCameraControl(ExposureControl.class);
                    //exposureControl.setMode(ExposureControl.Mode.Manual);
                    timeToCameraInit = time;
                    initialized = true;
                } catch (Exception e) {
                    telemetry.addLine("camera isn't initialized yet");
                }
            } else if(!propLocationOverride) {
                telemetry.addData("exposure", exposureControl.getExposure(TimeUnit.MILLISECONDS));
                /* if(gamepad1.right_trigger > 0)
                    exposure++;
                else if(gamepad1.left_trigger > 0)
                    exposure--;
                exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS); */

                propDetector.update();
                propLocation = propDetector.getLocation();
                telemetry.addLine();
                propDetector.telemetryAll(telemetry);
            }

            telemetry.update();
            sleep(10);
        }

        // start of op mode
        try {
            propDetector.stopDetecting();
        } catch(Exception e) {
            telemetry.addData("Camera not initialized", e);
        }
        //telemetry.setMsTransmissionInterval(250);

        telemetry.addData("Starting", "%s %s", alliance, side);
        telemetry.addData("Prop location", propLocation);
        telemetry.update();

        // trajectories
        Pose2d startPose, backdropPose = null, waitPose = null, parkPose;
        Vector2d enRouteToBackdrop = null;
        TrajectorySequence spikeMarkTraj = null, toBackdrop, parkTraj;
        double spikeMarkbackDistance = 15, backdropBackDistance = 8;

        if(alliance == Alliance.BLUE && side == Side.BACK) {
            // BLUE BACK (left)
            startPose = new Pose2d(12, 62.5, -Math.PI / 2);
            double backdropY = 0;
            parkPose = new Pose2d(64, propLocation == Location.RIGHT ? 65 : 67, 0);
            switch(propLocation) {
                case LEFT:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(22, 40, -Math.PI/2), -Math.PI/2)
                            .build();
                    backdropY = 45;
                    break;
                case CENTER:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(13, 34, -Math.PI/2), -Math.PI/2)
                            .build();
                    backdropY = 39;
                    break;
                case RIGHT:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(20, 39, -3*Math.PI/4), -3*Math.PI/4)
                            .splineToLinearHeading(new Pose2d(11, 32, Math.PI), Math.PI)
                            .build();
                    backdropY = 21;
            }
            backdropPose = new Pose2d(propLocation == Location.RIGHT ? 53 : 52, backdropY, 0);
        }
        else if(alliance == Alliance.BLUE && side == Side.FRONT) {
            // BLUE FRONT (right)
            startPose = new Pose2d(-36, 62, -Math.PI / 2);
            waitPose = new Pose2d(-36, 14, 0);
            enRouteToBackdrop = new Vector2d(10, 14);
            parkPose = new Pose2d(56, 14, 0);
            switch(propLocation) {
                case LEFT:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-20, 39, -Math.PI/4), -Math.PI/4)
                            .splineToLinearHeading(new Pose2d(0, 30, 0), 0)
                            .build();
                    break;
                case CENTER:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-15, 21, -Math.PI/2), -Math.PI/2)
                            .build();
                    break;
                case RIGHT:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-24, 31, -Math.PI/2), -Math.PI/2)
                            .build();
            }
        }
        else if(alliance == Alliance.RED && side == Side.BACK) {
            // RED BACK (right)
            startPose = new Pose2d(12, -61, Math.PI / 2);
            double backdropY = 0;
            parkPose = new Pose2d(56, -61, 0);
            switch(propLocation) {
                case LEFT:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(16, -47, 3*Math.PI/4), 3*Math.PI/4)
                            .splineToLinearHeading(new Pose2d(10, -26, Math.PI), Math.PI)
                            .build();
                    backdropY = -35;
                    break;
                case CENTER:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(17, -32, Math.PI/2), Math.PI/2)
                            .build();
                    backdropY = -40;
                    break;
                case RIGHT:
                    spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(24.5, -42, Math.PI/2), Math.PI/2)
                            .build();
                    backdropY = -45;
            }
            backdropPose = new Pose2d(51, backdropY, 0);
        }
        else {
            // RED FRONT (left)
            startPose = new Pose2d(-36, -61, Math.PI / 2);
            parkPose = new Pose2d(60, -11, Math.PI);
        }

        if(side == Side.BACK) {
            toBackdrop = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                    .addTemporalMarker(() -> {
                        robot.rotator.rotateFully();
                        robot.outtakeRaiser.goToPosition1(this);
                    })
                    .back(spikeMarkbackDistance)
                    .splineToLinearHeading(backdropPose, backdropPose.getHeading())
                    .build();
            parkTraj = robot.drive.trajectorySequenceBuilder(toBackdrop.end())
                    .back(backdropBackDistance)
                    .addTemporalMarker(1, () -> {
                        robot.rotator.retractFully();
                        robot.outtakeRaiser.goDown(this);
                    })
                    .splineToLinearHeading(parkPose, parkPose.getHeading()) // park in backstage
                    .build();
        } else {
            toBackdrop = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                    .back(spikeMarkbackDistance)
                    .splineToLinearHeading(waitPose, waitPose.getHeading())
                    .waitSeconds(3)
                    .lineTo(enRouteToBackdrop)
                    .addSpatialMarker(enRouteToBackdrop, () -> {
                        robot.rotator.rotateFully();
                        robot.outtakeRaiser.goToPosition1(this);
                    })
                    .splineToLinearHeading(backdropPose, backdropPose.getHeading())
                    .build();
            parkTraj = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                    .back(backdropBackDistance)
                    .splineToLinearHeading(parkPose, parkPose.getHeading())
                    .build();
        }

        robot.drive.setPoseEstimate(startPose);
        robot.gripper1.downFully();
        telemetry.addData("Status", "moving to spike mark");
        telemetry.update();
        robot.drive.followTrajectorySequence(spikeMarkTraj);
        telemetry.addData("Status", "releasing pixel");
        telemetry.update();
        robot.autoClaw.out();
        timer.reset();
        int intakeWaitTime = 1000;
        while(timer.milliseconds() < intakeWaitTime && opModeIsActive());
        robot.autoClaw.in();

        telemetry.addData("Status", "moving to backdrop");
        telemetry.update();
        robot.drive.followTrajectorySequence(toBackdrop);
        timer.reset();
        while(timer.milliseconds() < intakeWaitTime && opModeIsActive());
        robot.gripper1.upFully();
        robot.gripper2.upFully();
        robot.drive.followTrajectorySequence(parkTraj);
        timer.reset();
        while(timer.milliseconds() < intakeWaitTime && opModeIsActive());
    }
}