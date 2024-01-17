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
import org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.SampleMecanumDrive;
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
    private static Side side = Side.FRONT;
    private static boolean pickFromStack = false;

    Robot robot;
    TensorFlowObjectDetector propDetector;

    private Location propLocation = Location.LEFT;

    @Override
    public void runOpMode() {
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
        while (opModeInInit()) {
            if ((gamepad1.x) || gamepad2.x)
                alliance = Alliance.BLUE;
            else if ((gamepad1.b && !gamepad1.start) || (gamepad2.b && !gamepad2.start))
                alliance = Alliance.RED;
            else if (gamepad1.y || gamepad2.y)
                side = Side.BACK;
            else if ((gamepad1.a && !gamepad1.start) || (gamepad2.a && !gamepad2.start))
                side = Side.FRONT;
            else if (gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right || gamepad2.dpad_left || gamepad2.dpad_up || gamepad2.dpad_right)
                propLocationOverride = true;
            else if (gamepad1.back || gamepad2.back)
                propLocationOverride = false;
            else if ((gamepad1.start && gamepad1.back) || (gamepad2.start && gamepad2.back))
                requestOpModeStop();
            else if(gamepad1.right_bumper || gamepad2.right_bumper)
                pickFromStack = true;
            else if(gamepad1.left_bumper || gamepad2.left_bumper)
                pickFromStack = false;

            if (propLocationOverride) {
                if (gamepad1.dpad_left || gamepad2.dpad_left)
                    propLocation = Location.LEFT;
                else if (gamepad1.dpad_up || gamepad2.dpad_up)
                    propLocation = Location.CENTER;
                else if (gamepad1.dpad_right || gamepad2.dpad_right)
                    propLocation = Location.RIGHT;
            }

            telemetry.addLine(initialized ? "Initialized" : String.format(Locale.ENGLISH, "Initializing... %.1f", 3 - getRuntime()));
            telemetry.addData("Runtime", "%.1f", getRuntime());
            telemetry.addData("Camera init time", "%.2f", timeToCameraInit);
            telemetry.addData("\nAlliance", "%s (x = blue, b = red)", alliance);
            telemetry.addData("Side", "%s (a = front, y = back)", side);
            telemetry.addData("Pick from stack", pickFromStack);
            if (propLocationOverride)
                telemetry.addData("\nProp location", "%s (override)", propLocation);
            else
                telemetry.addData("\nProp location", initialized ? propLocation : null);

            if (!initialized && !propLocationOverride) {
                try {
                    exposureControl = propDetector.visionPortal.getCameraControl(ExposureControl.class);
                    //exposureControl.setMode(ExposureControl.Mode.Manual);
                    timeToCameraInit = time;
                    initialized = true;
                } catch (Exception e) {
                    telemetry.addLine("camera isn't initialized yet");
                }
            } else if (!propLocationOverride) {
                telemetry.addData("exposure mode", exposureControl.getMode());
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

        if(!opModeIsActive())
            return;

        try {
            propDetector.stopDetecting();
        } catch (Exception e) {
            telemetry.addData("Camera not initialized", e);
        }
        //telemetry.setMsTransmissionInterval(250);

        String run = String.format("Running %s %s %s, prop location = %s", alliance, side, pickFromStack ? "simple" : "", propLocation);
        telemetry.addLine(run);
        telemetry.update();

        // trajectories
        Pose2d startPose = null, backdropPose = null, stackPose = null, parkPose = null;
        Vector2d stageDoor = null;
        TrajectorySequence spikeMarkTraj = null, toBackdrop = null, parkTraj = null;
        TrajectorySequence backdropToStack = null, stackToBackdrop = null;
        double spikeMarkBackDistance = 2, backdropBackDistance = 5;
        double slowSpeed = 8;

        double backdropY = 0;
        switch (alliance) {
            case BLUE:
                switch(propLocation) {
                    case LEFT:
                        backdropY = 44;
                        break;
                    case CENTER:
                        backdropY = 39;
                        break;
                    case RIGHT:
                        backdropY = 26;
                }
                backdropPose = new Pose2d(45, backdropY, 0);
                stackPose = new Pose2d(-63, 23, 0);
                stageDoor = new Vector2d(-3, 7);
                break;
            case RED:
                switch(propLocation) {
                    case LEFT:
                        backdropY = -35;
                        break;
                    case CENTER:
                        backdropY = -40;
                        break;
                    case RIGHT:
                        backdropY = -45;
                }
                backdropPose = new Pose2d(51, backdropY, 0);
                stackPose = new Pose2d(-58, -35, 0);
                stageDoor = new Vector2d(-6, -11);
        }

        switch(side) {
            case BACK:
                switch(alliance) {
                    case BLUE: // BLUE BACK
                        startPose = new Pose2d(12, 62.5, -Math.PI / 2);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(22, 40, -Math.PI / 2), -Math.PI / 2)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(15, 34, -Math.PI / 2), -Math.PI / 2)
                                        .build();
                                break;
                            case RIGHT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        //.splineToSplineHeading(new Pose2d(20, 39, -3 * Math.PI / 4), -3 * Math.PI / 4)
                                        .forward(10)
                                        .splineToSplineHeading(new Pose2d(11, 32, Math.PI), Math.PI)
                                        .build();
                        }
                        parkPose = new Pose2d(64, propLocation == Location.RIGHT ? 65 : 68, 0);
                        break;
                    case RED: // RED BACK
                        startPose = new Pose2d(12, -61, Math.PI / 2);
                        parkPose = new Pose2d(56, -61, 0);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(16, -47, 3 * Math.PI / 4), 3 * Math.PI / 4)
                                        .splineToSplineHeading(new Pose2d(10, -26, Math.PI), Math.PI)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(17, -32, Math.PI / 2), Math.PI / 2)
                                        .build();
                                break;
                            case RIGHT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(24.5, -42, Math.PI / 2), Math.PI / 2)
                                        .build();
                        }
                }

                toBackdrop = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                        .addTemporalMarker(() -> {
                            robot.rotator.rotateFully();
                            robot.outtakeRaiser.goToUpPosition(this);
                        })
                        .back(spikeMarkBackDistance)
                        .splineToSplineHeading(backdropPose, backdropPose.getHeading())
                        .forward(3,
                                SampleMecanumDrive.getVelocityConstraint(slowSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                parkTraj = robot.drive.trajectorySequenceBuilder(toBackdrop.end())
                        .waitSeconds(1)
                        .back(backdropBackDistance)
                        .addDisplacementMarker(() -> {
                            robot.rotator.retractFully();
                            robot.outtakeRaiser.goDown(this);
                        })
                        .splineToLinearHeading(parkPose, parkPose.getHeading()) // park in backstage
                        .build();
                break;
            case FRONT:
                switch(alliance) {
                    case BLUE: // blue front
                        startPose = new Pose2d(-36, 62, -Math.PI / 2);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        //.splineToSplineHeading(new Pose2d(-39, 39, -Math.PI / 4), -Math.PI / 4)
                                        .forward(7)
                                        .splineToSplineHeading(new Pose2d(-33, 29, 0), 0)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-39, 20, 0), -Math.PI / 2)
                                        .back(5)
                                        .build();
                                break;
                            case RIGHT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-45, 28, Math.PI), -Math.PI / 2)
                                        .back(4)
                                        .build();
                        }
                        stackPose = new Pose2d(-56, 23, 0);
                        parkPose = new Pose2d(56, 14, 0);
                        break;
                    case RED: // red front
                        startPose = new Pose2d(-36, -61, Math.PI / 2);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-24, -31, Math.PI / 2), Math.PI / 2)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-28, -30, 0), Math.PI / 2)
                                        .build();
                                break;
                            case RIGHT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-38, -45, Math.PI / 4), Math.PI / 4)
                                        .splineToSplineHeading(new Pose2d(-36, -35, 0), 0)
                                        .build();
                        }
                        stackPose = new Pose2d(-56, -35, 0);
                        parkPose = new Pose2d(60, -11, Math.PI);
                }

                toBackdrop = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                        .back(spikeMarkBackDistance)
//                        .splineToSplineHeading(stackPose, Math.PI)
//                        .addDisplacementMarker(() -> {
//                            robot.intake.lower();
//                            robot.intake.in();
//                        })
//                        .back(3, SampleMecanumDrive.getVelocityConstraint(slowSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                        .waitSeconds(1)
                        .lineToConstantHeading(stageDoor.plus(new Vector2d(-18, 0)))
                        .addDisplacementMarker(() -> {
//                            robot.intake.stop();
//                            robot.claw1.down();
//                            robot.claw2.down();
                            telemetry.addLine(run);
                            telemetry.addData("Status", "waiting");
                            telemetry.update();
                        })
                        .waitSeconds(1)
                        .lineToConstantHeading(stageDoor.plus(new Vector2d(12, 0)))
                        .addDisplacementMarker(() -> {
                            telemetry.addLine(run);
                            telemetry.addData("Status", "moving to backdrop");
                            telemetry.update();
                            robot.rotator.rotateFully();
                            robot.outtakeRaiser.goToUpPosition(this);
                        })
                        .splineToSplineHeading(backdropPose, backdropPose.getHeading())
                        .waitSeconds(1)
                        .forward(3,
                                SampleMecanumDrive.getVelocityConstraint(slowSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                parkTraj = robot.drive.trajectorySequenceBuilder(toBackdrop.end())
                        .waitSeconds(1)
                        .back(backdropBackDistance)
                        .addDisplacementMarker(() -> {
                            robot.rotator.retractFully();
                            robot.outtakeRaiser.goDown(this);
                        })
                        .splineToLinearHeading(parkPose, parkPose.getHeading())
                        .build();
        }

        if(pickFromStack) {
            backdropToStack = robot.drive.trajectorySequenceBuilder(toBackdrop.end())
                    .back(3)
                    .addDisplacementMarker(() -> {
                        telemetry.addLine(run);
                        telemetry.addData("Status", "moving to stack");
                        telemetry.update();
                        robot.rotator.retractFully();
                        robot.outtakeRaiser.goDown(this);
                    })
                    .splineTo(stageDoor, Math.PI)
                    .splineToSplineHeading(stackPose, Math.PI)
                    .addDisplacementMarker(() -> {
                        telemetry.addLine(run);
                        telemetry.addData("Status", "picking from stack");
                        telemetry.update();
                        robot.intake.lower();
                        robot.intake.in();
                    })
                    .back(5, SampleMecanumDrive.getVelocityConstraint(slowSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            stackToBackdrop = robot.drive.trajectorySequenceBuilder(backdropToStack.end())
                    .splineTo(stageDoor, 0)
                    .addDisplacementMarker(() -> {
                        robot.intake.stop();
                        robot.claw1.down();
                        robot.claw2.down();
                        robot.rotator.rotateFully();
                        robot.outtakeRaiser.goToUpPosition(this);
                        telemetry.addLine(run);
                        telemetry.addData("Status", "moving to backdrop");
                        telemetry.update();
                    })
                    .splineToSplineHeading(backdropPose, 0)
                    .forward(5, SampleMecanumDrive.getVelocityConstraint(slowSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
        }

        robot.drive.setPoseEstimate(startPose);
        robot.claw1.down();

        telemetry.addLine(run);
        telemetry.addData("Status", "moving to spike mark");
        telemetry.update();
        robot.drive.followTrajectorySequence(spikeMarkTraj);

        telemetry.addLine(run);
        telemetry.addData("Status", "releasing pixel");
        telemetry.update();
        robot.autoClaw.out();
        timer.reset();
        int intakeWaitTime = 1000;
        while(timer.milliseconds() < intakeWaitTime && opModeIsActive());
        robot.autoClaw.in();

        telemetry.addLine(run);
        telemetry.addData("Status", "moving to backdrop");
        telemetry.update();
        robot.drive.followTrajectorySequence(toBackdrop);

        telemetry.addLine(run);
        telemetry.addData("Status", "placing pixel");
        telemetry.update();
        timer.reset();
        while(timer.milliseconds() < intakeWaitTime && opModeIsActive());
        robot.claw1.up();
        robot.claw2.up();

//        telemetry.addLine(run);
//        telemetry.addData("Status", "parking");
//        telemetry.update();
//        robot.drive.followTrajectorySequence(parkTraj);
//
//        telemetry.addLine(run);
//        telemetry.addData("Status", "parked");
//        telemetry.update();
        timer.reset();
        while(timer.seconds() < 2 && opModeIsActive());
//        robot.outtakeRaiser.stop();
//        while(opModeIsActive());
    }
}