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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.Alliance;
import org.firstinspires.ftc.teamcode.Auto.Location;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "old (no april tags)", group = "auto", preselectTeleOp = "TeleOp")
public class TensorFlowAuto extends LinearOpMode
{
    private enum Side {
        BACK, FRONT
    }

    private static Alliance alliance = Alliance.BLUE;
    private static Side side = Side.BACK;
    private static boolean placeOnBackdrop = true, goThroughStageDoor = true;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        TensorFlowObjectDetector propDetector = new TensorFlowObjectDetector(hardwareMap);
        Location propLocation = Location.LEFT;
        boolean propLocationOverride = false;
        ElapsedTime timer = new ElapsedTime();
        boolean initialized = false;
        double initTime = 5;

        // init loop - select alliance and side
        while (opModeInInit()) {
            if(!initialized && getRuntime() > initTime)
                initialized = true;

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
            else if(gamepad1.right_bumper || gamepad2.right_bumper)
                placeOnBackdrop = true;
            else if(gamepad1.left_bumper || gamepad2.left_bumper) {
                placeOnBackdrop = false;
                if(side == Side.FRONT)
                    goThroughStageDoor = false;
            } else if(gamepad1.right_trigger > .5 || gamepad2.right_trigger > .5)
                goThroughStageDoor = true;
            else if(gamepad1.left_trigger > .5 || gamepad2.left_trigger > .5)
                goThroughStageDoor = false;
            else if ((gamepad1.start && gamepad1.back) || (gamepad2.start && gamepad2.back))
                requestOpModeStop();

            if (propLocationOverride) {
                if (gamepad1.dpad_left || gamepad2.dpad_left)
                    propLocation = Location.LEFT;
                else if (gamepad1.dpad_up || gamepad2.dpad_up)
                    propLocation = Location.CENTER;
                else if (gamepad1.dpad_right || gamepad2.dpad_right)
                    propLocation = Location.RIGHT;
            }

            telemetry.addData(initialized ? "Initialized" : "Initializing please wait...", "%.1f", Math.max(initTime - getRuntime(), 0));
            telemetry.addLine().addData("Runtime", "%.1f", getRuntime()).addData("loop time", "%.2f ms", timer.milliseconds());
            timer.reset();
            telemetry.addData("\nAlliance", "%s (x = blue, b = red)", alliance);
            telemetry.addData("Side", "%s (a = front, y = back)", side);

            if(side == Side.FRONT)
                telemetry.addData("Go through stage door (RT/LT)", goThroughStageDoor);
            telemetry.addData("Place on backdrop (RB/LB)", placeOnBackdrop);

            if (propLocationOverride)
                telemetry.addData("\nProp location", "%s (override)", propLocation);
            else
                telemetry.addData("\nProp location", propLocation);

            if (!propLocationOverride) {
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

        robot.claw1.down();
        robot.claw2.down();

        initTime = getRuntime();
        double waitTime;
        if(side == Side.BACK || !placeOnBackdrop)
            waitTime = Math.max(5 - initTime, 0);
        else
            waitTime = 4;
        resetRuntime();
        while(getRuntime() < waitTime) {
            if(!propLocationOverride) {
                telemetry.addLine(String.format("Detecting prop location...%.1f", waitTime - getRuntime()));
                propDetector.update();
                propLocation = propDetector.getLocation();
                telemetry.addData("Prop location", propLocation);
                telemetry.addLine();
                propDetector.telemetryAll(telemetry);
                telemetry.update();
            } else {
                telemetry.addLine(String.format("Waiting...%.1f", waitTime - getRuntime()));
                telemetry.addData("Prop location", propLocation);
            }
        }

        propDetector.stopDetecting();
        //telemetry.setMsTransmissionInterval(250);

        telemetry.setAutoClear(false);
        telemetry.addData("Running", "%s %s,  prop location = %s", alliance, side, propLocation);
        if(side == Side.FRONT)
            telemetry.addData("Go through stage door", goThroughStageDoor);
        telemetry.addData("Place on backdrop", placeOnBackdrop);
        Telemetry.Item status = telemetry.addData("State", "loading...");
        telemetry.update();

        // trajectories
        Pose2d startPose = null, backdropPose = null, stackPose = null, parkPose = null;
        Vector2d truss = null;
        TrajectorySequence spikeMarkTraj = null, toBackdrop = null, parkTraj = null;
        double spikeMarkBackDistance = 2-.5, backdropBackDistance = 5;
        double slowSpeed = 2;

        double backdropX, backdropY = 0;
        switch (alliance) {
            case BLUE: // blue common
                backdropX = 49.5+.3;
                switch(propLocation) { // blue backdrop
                    case LEFT:
                        backdropX -= side == Side.BACK ? 1.8-.4 : 1.8;
                        backdropY = side == Side.BACK ? 39.1 : 41.3;
                        break;
                    case CENTER:
//                        backdropX -= .7-.5;
                        backdropY = side == Side.BACK ? 32 : 33;
                        break;
                    case RIGHT:
                        backdropX -= 3.5;
                        backdropY = side == Side.BACK ? 30.1 : 25;
                }
                backdropPose = new Pose2d(backdropX, backdropY, 0);
                stackPose = new Pose2d(-56, 23, 0);
                truss = goThroughStageDoor ? new Vector2d(-3, 10-.5) : new Vector2d(-3, 63);
                break;
            case RED: // red common
                backdropX = side == Side.BACK ? 48.6 : 49.4+.6;
                switch(propLocation) { // red backdrop
                    case LEFT:
                        backdropX += 1;
                        backdropY = side == Side.BACK ? -27 : -28.3+.6;
                        break;
                    case CENTER:
                        backdropX += side == Side.BACK ? 2.7 : 3.1;
                        backdropY = side == Side.BACK ? -30 : -28+2;
                        break;
                    case RIGHT:
                        backdropX += 2.7;
                        backdropY = -36.2;
                }
                backdropPose = new Pose2d(backdropX, backdropY, Math.toRadians(0));
                stackPose = new Pose2d(-56, -35, 0);
                truss = goThroughStageDoor ? new Vector2d(-6, -6) : new Vector2d(-6, -60);
        }

        switch(side) {
            case BACK:
                switch(alliance) {
                    case BLUE: // BLUE BACK
                        startPose = new Pose2d(12, 64, -Math.PI / 2);
                        parkPose = new Pose2d(52+3, 62, 0);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(23.5, 40, -Math.PI / 2), -Math.PI / 2)
                                        .back(2.5)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(15, 31+.7, -Math.PI / 2), -Math.PI / 2)
                                        .back(4.5)
                                        .build();
                                break;
                            case RIGHT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        //.splineToSplineHeading(new Pose2d(20, 39, -3 * Math.PI / 4), -3 * Math.PI / 4)
                                        .forward(10)
                                        .splineToSplineHeading(new Pose2d(7, 36, Math.PI), Math.PI)
                                        .back(2)
                                        .build();
                                parkPose = parkPose.plus(new Pose2d(0, 2+1, 0));
                        }
                        break;
                    case RED: // RED BACK
                        startPose = new Pose2d(12, -61, Math.PI / 2);
                        parkPose = new Pose2d(47, -59, 0);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .forward(13)
                                        .splineToSplineHeading(new Pose2d(7, -34, Math.PI), Math.PI)
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
                                        .splineToSplineHeading(new Pose2d(21.6+.4, -34, Math.PI / 2), Math.PI / 2)
                                        .back(4)
                                        .build();
                        }
                }

                // back traj
                toBackdrop = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                        .addTemporalMarker(() -> {
                            robot.outtake.rotator.rotateFully();
                            robot.outtake.goToUpPosition();
                        })
                        .back(spikeMarkBackDistance)
                        .splineToSplineHeading(backdropPose, backdropPose.getHeading())
                        .waitSeconds(1)
                        .forward(4-.5,
                                SampleMecanumDrive.getVelocityConstraint(slowSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                if(placeOnBackdrop) {
                    parkTraj = robot.drive.trajectorySequenceBuilder(toBackdrop.end())
                            .waitSeconds(1)
                            .back(backdropBackDistance, SampleMecanumDrive.getVelocityConstraint(slowSpeed * 5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addDisplacementMarker(() -> {
                                robot.outtake.rotator.retractFully();
                                robot.outtake.goToDownPosition();
                            })
                            .splineToLinearHeading(parkPose, (alliance == Alliance.BLUE ? 1 : -1) * Math.PI / 4)
                            .build();
                } else { // !placeOnBackdrop
                    parkTraj = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                            .back(spikeMarkBackDistance)
                            .splineToSplineHeading(parkPose, (alliance == Alliance.BLUE ? 1 : -1) * Math.PI/4)
                            .build();
                }
                break;
            case FRONT:
                switch(alliance) {
                    case BLUE: // blue front
                        startPose = new Pose2d(-36, 62, -Math.PI / 2);
                        parkPose = goThroughStageDoor ? new Pose2d(46, 14, 0) : new Pose2d(52, 62, 0);
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
                                        .splineToSplineHeading(new Pose2d(-40, 22, 0), 0)
                                        .back(2+1)
                                        .build();
                                spikeMarkBackDistance += 3;
                                break;
                            case RIGHT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-45, 28, Math.PI), -Math.PI / 2)
                                        .back(5)
                                        .build();
                                spikeMarkBackDistance = 1;
                        }
                        break;
                    case RED: // red front
                        startPose = new Pose2d(-36, -61, Math.PI / 2);
                        parkPose = goThroughStageDoor ? new Pose2d(52, -10, 0) : new Pose2d(47, -59, 0);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-39.5-1, -31, Math.PI), Math.PI / 2)
                                        .back(4+1)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-40, -21.8, 0), Math.PI / 2)
                                        .build();
                                break;
                            case RIGHT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .forward(7)
                                        .splineToSplineHeading(new Pose2d(-31.8, -33, 0), 0)
                                        .back(2)
                                        .build();
                        }
                }

                // front traj
                waitTime = placeOnBackdrop ? 12-2 - waitTime : 0;
                if(alliance == Alliance.BLUE && propLocation == Location.CENTER && goThroughStageDoor)
                    toBackdrop = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                            .back(spikeMarkBackDistance)
                            .splineToConstantHeading(truss.plus(new Vector2d(-46, 3)), -Math.PI/2)
                            .splineToConstantHeading(truss.plus(new Vector2d(-33, 0)), 0)
                            .waitSeconds(waitTime)
                            .lineToConstantHeading(truss.plus(new Vector2d(12+21, 0)))
                            .addDisplacementMarker(() -> {
                                robot.outtake.rotator.rotateFully();
                                robot.outtake.goToPosition(Outtake.POSITION_2);
                            })
                            .splineToSplineHeading(backdropPose, backdropPose.getHeading())
                            .waitSeconds(1)
                            .forward(4.3,
                                    SampleMecanumDrive.getVelocityConstraint(slowSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();
                else
                    toBackdrop = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                        .back(spikeMarkBackDistance)
                        .lineToConstantHeading(truss.plus(new Vector2d(-35, 0)))
                        .waitSeconds(waitTime)
                        .lineToConstantHeading(truss.plus(new Vector2d(12+21, 0)))
                        .addDisplacementMarker(() -> {
                            robot.outtake.rotator.rotateFully();
                            robot.outtake.goToUpPosition();
                        })
                        .splineToSplineHeading(backdropPose, backdropPose.getHeading())
                        .waitSeconds(1)
                        .forward(4.3,
                                SampleMecanumDrive.getVelocityConstraint(slowSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                if(placeOnBackdrop) {
                    parkTraj = robot.drive.trajectorySequenceBuilder(toBackdrop.end())
                            .waitSeconds(1)
                            .back(backdropBackDistance, SampleMecanumDrive.getVelocityConstraint(slowSpeed * 3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addTemporalMarker(2, () -> {
                                robot.outtake.rotator.retractFully();
                                robot.outtake.goToDownPosition();
                            })
                            .forward(1, SampleMecanumDrive.getVelocityConstraint(slowSpeed * 3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();
                } else
                    parkTraj = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                            .back(spikeMarkBackDistance)
                            .lineToConstantHeading(truss.plus(new Vector2d(-33, 0)))
                            .lineToConstantHeading(truss.plus(new Vector2d(12, 0)))
                            .splineToSplineHeading(parkPose, 0)
                            .build();
        }

        robot.drive.setPoseEstimate(startPose);

        status.setValue("moving to spike mark");
        telemetry.update();
        robot.drive.followTrajectorySequence(spikeMarkTraj);

        status.setValue("releasing pixel");
        telemetry.update();
        robot.autoClaw.out();
        timer.reset();
        int intakeWaitTime = 1000;
        while(timer.milliseconds() < intakeWaitTime && opModeIsActive());
        robot.autoClaw.in();

        if(placeOnBackdrop) {
            status.setValue("moving to backdrop");
            telemetry.update();
            robot.drive.followTrajectorySequence(toBackdrop);

            status.setValue("placing pixel");
            telemetry.update();
            timer.reset();
            while (timer.seconds() < 1 && opModeIsActive()) ;
            robot.claw1.up();
            robot.claw2.up();
            while (timer.seconds() < 2 && opModeIsActive()) ;
            robot.outtake.goToPosition(robot.outtake.getPosition() + 200, .3);
            while (!robot.outtake.isIdle() && timer.seconds() < 4 && opModeIsActive()) ;
        }

        status.setValue("parking");
        telemetry.update();
        robot.drive.followTrajectorySequence(parkTraj);

        status.setValue("parked");
        telemetry.update();
        timer.reset();
        while(timer.seconds() < 2 && opModeIsActive());
        robot.outtake.stop();

        robot.intake.raise();

        timer.reset();
        while(timer.seconds() < 2 && opModeIsActive());
    }
}