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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.Auto.Alliance;
import org.firstinspires.ftc.teamcode.Auto.Location;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "auto", preselectTeleOp = "TeleOp")
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
        while (opModeInInit() || !initialized) {
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

            telemetry.addLine(initialized ? "Initialized" : String.format(Locale.ENGLISH, "Initializing please wait... %.1f", Math.max(3 - getRuntime(), 0)));
            telemetry.addLine().addData("Runtime", "%.1f", getRuntime()).addData("loop time", "%.2f ms", timer.milliseconds());
            timer.reset();
            telemetry.addData("\nAlliance", "%s (x = blue, b = red)", alliance);
            telemetry.addData("Side", "%s (a = front, y = back)", side);
//            if(initialized)
//                telemetry.addData("Camera init time", "%.2f", timeToCameraInit);
            if(pickFromStack)
                telemetry.addData("Pick from stack", pickFromStack);
            if (propLocationOverride)
                telemetry.addData("\nProp location", "%s (override)", propLocation);
            else
                telemetry.addData("\nProp location", initialized ? propLocation : null);

            if (!initialized) {
                try {
                    exposureControl = propDetector.visionPortal.getCameraControl(ExposureControl.class);
//                    if(exposureControl.isModeSupported(ExposureControl.Mode.Manual))
//                        exposureControl.setMode(ExposureControl.Mode.Manual);
//                    timeToCameraInit = time;
                    initialized = true;
                } catch (Exception e) {
                    telemetry.addLine("camera isn't initialized yet");
                }
            } else if (!propLocationOverride) {
                telemetry.addData("exposure mode", exposureControl.getMode()); // default is AperturePriority
                telemetry.addData("exposure", exposureControl.getExposure(TimeUnit.MILLISECONDS));
                /* if(gamepad1.right_trigger > .5)
                    exposure++;
                else if(gamepad1.left_trigger > .5)
                    exposure--;
                exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS); */

                propDetector.alliance = alliance;
                propDetector.update();
                propLocation = propDetector.getLocation();
                telemetry.addLine();
                propDetector.telemetryAll(telemetry);
            }

            telemetry.update();
            //sleep(10);
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

        telemetry.setAutoClear(false);
        telemetry.addLine(String.format("Running %s %s %s, prop location = %s", alliance, side, pickFromStack ? "simple" : "", propLocation));
        Telemetry.Item status = telemetry.addData("Status", "loading...");
        telemetry.update();

        // trajectories
        Pose2d startPose = null, backdropPose = null, stackPose = null, parkPose = null;
        Vector2d stageDoor = null;
        TrajectorySequence spikeMarkTraj = null, toBackdrop = null, parkTraj = null;
        TrajectorySequence backdropToStack = null, stackToBackdrop = null;
        double spikeMarkBackDistance = 2, backdropBackDistance = 5;
        double slowSpeed = 5;

        double backdropY = 0;
        switch (alliance) {
            case BLUE: // blue common
                switch(propLocation) { // blue backdrop
                    case LEFT:
                        backdropY = 41-.5;
                        break;
                    case CENTER:
                        backdropY = side == Side.BACK ? 34 : 35.5;
                        break;
                    case RIGHT:
                        backdropY = side == Side.BACK ? 31 : 31-2;
                }
                backdropPose = new Pose2d(47, backdropY, 0);
                stackPose = new Pose2d(-56, 23, 0);
                stageDoor = new Vector2d(-3, 10);
                break;
            case RED: // red common
                switch(propLocation) { // red backdrop
                    case LEFT:
                        backdropY = -33+2;
                        break;
                    case CENTER:
                        backdropY = -38+1;
                        break;
                    case RIGHT:
                        backdropY = -42;
                }
                backdropPose = new Pose2d(45, backdropY, 0);
                stackPose = new Pose2d(-56, -35, 0);
                stageDoor = new Vector2d(-6, -9);
        }

        switch(side) {
            case BACK:
                switch(alliance) {
                    case BLUE: // BLUE BACK
                        startPose = new Pose2d(12, 64, -Math.PI / 2);
                        parkPose = new Pose2d(52, 65, 0);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(23+1, 40, -Math.PI / 2), -Math.PI / 2)
                                        .back(2.5)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(15, 31, -Math.PI / 2), -Math.PI / 2)
                                        .back(4.5)
                                        .build();
                                break;
                            case RIGHT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        //.splineToSplineHeading(new Pose2d(20, 39, -3 * Math.PI / 4), -3 * Math.PI / 4)
                                        .forward(10)
                                        .splineToSplineHeading(new Pose2d(7, 36, Math.PI), Math.PI)
                                        .back(2.5-.5)
                                        .build();
                        }
                        break;
                    case RED: // RED BACK
                        startPose = new Pose2d(12, -61, Math.PI / 2);
                        parkPose = new Pose2d(47, -61, 0);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .forward(13)
                                        .splineToSplineHeading(new Pose2d(7, -33, Math.PI), Math.PI)
                                        .back(2)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(16, -30, Math.PI / 2), Math.PI / 2)
                                        .back(2)
                                        .build();
                                break;
                            case RIGHT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(24.5, -34, Math.PI / 2), Math.PI / 2)
                                        .back(2)
                                        .build();
                        }
                }

                toBackdrop = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                        .addTemporalMarker(() -> {
                            robot.outtake.rotator.rotateFully();
                            robot.outtake.goToUpPosition();
                        })
                        .back(spikeMarkBackDistance)
                        .splineToSplineHeading(backdropPose, backdropPose.getHeading())
                        .waitSeconds(1)
                        .forward(4,
                                SampleMecanumDrive.getVelocityConstraint(slowSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                break;
            case FRONT:
                switch(alliance) {
                    case BLUE: // blue front
                        startPose = new Pose2d(-36, 62, -Math.PI / 2);
                        parkPose = new Pose2d(46, 14, 0);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .forward(7)
                                        .splineToSplineHeading(new Pose2d(-31, 34, 0), 0)
                                        .back(3)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-47+8, 24, 0), -Math.PI / 2)
                                        .back(1)
                                        .build();
                                spikeMarkBackDistance += 2;
                                break;
                            case RIGHT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-45, 28, Math.PI), -Math.PI / 2)
                                        .back(4.5)
                                        .build();
                        }
                        break;
                    case RED: // red front
                        startPose = new Pose2d(-36, -61, Math.PI / 2);
                        parkPose = new Pose2d(52, -10, 0);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-39, -31, Math.PI), Math.PI / 2)
                                        .back(2)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-40, -30, 0), Math.PI / 2)
                                        .build();
                                break;
                            case RIGHT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .forward(7)
                                        .splineToSplineHeading(new Pose2d(-32, -35, 0), 0)
                                        .back(2)
                                        .build();
                        }
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
                        .lineToConstantHeading(stageDoor.plus(new Vector2d(-33, 0)))
//                        .addDisplacementMarker(() -> {
////                            robot.intake.stop();
////                            robot.claw1.down();
////                            robot.claw2.down();
//                        })
                        //.waitSeconds(1)
                        .lineToConstantHeading(stageDoor.plus(new Vector2d(12, 0)))
                        .addDisplacementMarker(() -> {
//                            telemetry.addLine(run);
//                            telemetry.addData("Status", "moving to backdrop");
//                            telemetry.update();
                            robot.outtake.rotator.rotateFully();
                            robot.outtake.goToUpPosition();
                        })
                        .splineToSplineHeading(backdropPose, backdropPose.getHeading())
                        .waitSeconds(1)
                        .forward(3,
                                SampleMecanumDrive.getVelocityConstraint(slowSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
        }

        parkTraj = robot.drive.trajectorySequenceBuilder(toBackdrop.end())
                .waitSeconds(1)
                .back(backdropBackDistance, SampleMecanumDrive.getVelocityConstraint(slowSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.outtake.rotator.retractFully();
                    robot.outtake.goToDownPosition();
                })
                .splineToLinearHeading(parkPose,
                        (alliance == Alliance.BLUE && side == Side.BACK) || (alliance == Alliance.RED && side == Side.FRONT) ? Math.PI/2 : -Math.PI/2)
                .build();

        if(pickFromStack) {
            backdropToStack = robot.drive.trajectorySequenceBuilder(toBackdrop.end())
                    .back(3)
                    .addDisplacementMarker(() -> {
                        status.setValue("Status", "moving to stack");
                        telemetry.update();
                        robot.outtake.rotator.retractFully();
                        robot.outtake.goToDownPosition();
                    })
                    .splineTo(stageDoor, Math.PI)
                    .splineToSplineHeading(stackPose, Math.PI)
                    .addDisplacementMarker(() -> {
                        status.setValue("Status", "picking from stack");
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
                        robot.outtake.rotator.rotateFully();
                        robot.outtake.goToUpPosition();
                        status.setValue("Status", "moving to backdrop");
                        telemetry.update();
                    })
                    .splineToSplineHeading(backdropPose, 0)
                    .forward(5, SampleMecanumDrive.getVelocityConstraint(slowSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
        }

        robot.drive.setPoseEstimate(startPose);
        robot.claw1.down();

        status.setValue("Status", "moving to spike mark");
        telemetry.update();
        robot.drive.followTrajectorySequence(spikeMarkTraj);

        status.setValue("Status", "releasing pixel");
        telemetry.update();
        robot.autoClaw.out();
        timer.reset();
        int intakeWaitTime = 1000;
        while(timer.milliseconds() < intakeWaitTime && opModeIsActive());
        robot.autoClaw.in();

        status.setValue("Status", "moving to backdrop");
        telemetry.update();
        robot.drive.followTrajectorySequence(toBackdrop);

        status.setValue("Status", "placing pixel");
        telemetry.update();
        timer.reset();
        while(timer.seconds() < 1 && opModeIsActive());
        robot.claw1.up();
        robot.claw2.up();
        while(timer.seconds() < 2 && opModeIsActive());
        robot.outtake.goToPosition(robot.outtake.getPosition() + 200, .5);
        while(!robot.outtake.isIdle() && timer.seconds() < 4 && opModeIsActive());

        status.setValue("Status", "parking");
        telemetry.update();
        robot.drive.followTrajectorySequence(parkTraj);

        status.setValue("Status", "parked");
        telemetry.update();
        timer.reset();
        while(timer.seconds() < 3 && opModeIsActive());
        robot.outtake.stop();
        //while(opModeIsActive());
    }
}