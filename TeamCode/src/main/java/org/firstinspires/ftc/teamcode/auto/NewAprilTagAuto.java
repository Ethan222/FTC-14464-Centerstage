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

package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp;
import org.firstinspires.ftc.teamcode.auto.AprilTags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.AprilTags.AprilTagIDs;
import org.firstinspires.ftc.teamcode.auto.AprilTags.Backdrop;
import org.firstinspires.ftc.teamcode.auto.TensorFlow.TensorFlowObjectDetector;
import org.firstinspires.ftc.teamcode.auto.enums.Alliance;
import org.firstinspires.ftc.teamcode.auto.enums.Location;
import org.firstinspires.ftc.teamcode.auto.enums.Side;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "New auto", group = "auto", preselectTeleOp = "TeleOp")
public class NewAprilTagAuto extends LinearOpMode
{
    private static Alliance alliance = Alliance.BLUE;
    private static Side side = Side.BACK;
    private static boolean goThroughStageDoor = true, placeOnBackdrop = true, useAprilTags = true;

    private Location propLocation = Location.LEFT;

    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private static final double INCHES_PER_METER = 13.12336;
    private Backdrop backdrop;
    private AprilTagDetection tagOfInterest = null;
    private List<AprilTagDetection> currentAprilTagDetections;
    private List<Location> detectedTags;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        TensorFlowObjectDetector propDetector = new TensorFlowObjectDetector(hardwareMap);
        ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();

        ElapsedTime timer = new ElapsedTime();
        boolean propLocationOverride = false;
        boolean initialized = false;
        double minInitTime = 5;
        Gamepad gamepad = gamepad1;

        telemetry.setAutoClear(true);
        Telemetry.Item status = telemetry.addData("Status", null).setRetained(true);
        telemetry.addLine().addData("Runtime", "%.1f", this::getRuntime).addData("loop time", "%.2f ms", timer::milliseconds).setRetained(true);
        Telemetry.Item allianceTelemetry = telemetry.addData("Alliance", alliance).setRetained(true);
        Telemetry.Item sideTelemetry = telemetry.addData("Side", side).setRetained(true);
        Telemetry.Item stageDoorTelemetry = telemetry.addData("Go through stage door (RT/LT)", goThroughStageDoor).setRetained(true);
        Telemetry.Item placeOnBackdropTelemetry = telemetry.addData("Place on backdrop (RB/LB)", placeOnBackdrop).setRetained(true);
        String useAprilTagsCaption = "Use april tags (RSB/LSB)";
        Telemetry.Item useAprilTagsTelemetry = useAprilTags ? telemetry.addData(useAprilTagsCaption, useAprilTags).setRetained(true) : null;
        Telemetry.Item propLocationTelemetry = telemetry.addData("\nProp location", propLocation).setRetained(true);

        // init loop todo
        while(opModeInInit() && !(gamepad.start && gamepad.back)) {
            if(!initialized && getRuntime() > minInitTime) {
                initialized = true;
                status.setValue("initialized");
            }
            if(!initialized)
                status.setValue("initializing please wait...%.1f", minInitTime - getRuntime());

            timer.reset();

            // gamepad input
            {
                if (!gamepad1.atRest() && gamepad != gamepad1)
                    gamepad = gamepad1;
                else if (!gamepad2.atRest() && gamepad != gamepad2)
                    gamepad = gamepad2;

                if(gamepad.x || (gamepad.b && !gamepad.start)) {
                    alliance = gamepad.x ? Alliance.BLUE : Alliance.RED;
                    allianceTelemetry.setValue(alliance);
                }

                if(gamepad.y || (gamepad.a && !gamepad.start)) {
                    if (side != Side.BACK && gamepad.y) {
                        side = Side.BACK;
                        stageDoorTelemetry.setValue("N/A");
                    } else if(side != Side.FRONT && (gamepad.a && !gamepad.start))
                        side = Side.FRONT;
                    sideTelemetry.setValue(side);
                }

                if(side == Side.FRONT && (gamepad.right_trigger > .5 || gamepad.left_trigger > .5)) {
                    goThroughStageDoor = (gamepad.right_trigger > .5);
                    stageDoorTelemetry.setValue(goThroughStageDoor);
                }

                if(gamepad.right_bumper || gamepad.left_bumper) {
                    if (!placeOnBackdrop && gamepad.right_bumper) {
                        placeOnBackdrop = true;
                        useAprilTagsTelemetry = telemetry.addData(useAprilTagsCaption, useAprilTags).setRetained(true);
                    } else if(placeOnBackdrop && gamepad.left_bumper) {
                        placeOnBackdrop = false;
                        telemetry.removeItem(useAprilTagsTelemetry);
                        if (side == Side.FRONT) {
                            goThroughStageDoor = false;
                            stageDoorTelemetry.setValue(false);
                        }
                    }
                    placeOnBackdropTelemetry.setValue(placeOnBackdrop);
                }

                if(placeOnBackdrop && gamepad.right_stick_button || gamepad.left_stick_button) {
                    useAprilTags = gamepad.right_stick_button;
                    useAprilTagsTelemetry.setValue(useAprilTags);
                }

                if (gamepad.dpad_left || gamepad.dpad_up || gamepad.dpad_right)
                    propLocationOverride = true;
                else if (gamepad.back)
                    propLocationOverride = false;

                if (propLocationOverride && (gamepad.dpad_left || gamepad.dpad_up || gamepad.dpad_right)) {
                    if (gamepad.dpad_left)
                        propLocation = Location.LEFT;
                    else if (gamepad.dpad_up)
                        propLocation = Location.CENTER;
                    else
                        propLocation = Location.RIGHT;
                    propLocationTelemetry.setValue(propLocation + " (override)");
                }
            }

            if (!propLocationOverride) {
                propDetector.alliance = alliance;
                propDetector.update();
                propLocation = propDetector.getLocation();
                propLocationTelemetry.setValue(propLocation);
                telemetry.addLine();
                propDetector.telemetryAll(telemetry);
            }

            telemetry.update();
        }

        // start of op mode todo
        if(!opModeIsActive())
            return;

        robot.claw1.down();
        robot.claw2.down();

        telemetry.clearAll();
        telemetry.setAutoClear(true);
        double waitTime = Math.max(5 - getRuntime(), 0);
        resetRuntime();
        while(getRuntime() < waitTime) {
            if(!propLocationOverride) {
                telemetry.addLine(String.format("Detecting prop location...%.1f", waitTime - getRuntime()));
                propDetector.update();
                propLocation = propDetector.getLocation();
                telemetry.addData("Prop location", propLocation);
                telemetry.addLine();
                propDetector.telemetryAll(telemetry);
            } else {
                telemetry.addLine(String.format("Waiting...%.1f", waitTime - getRuntime()));
                telemetry.addData("Prop location", propLocation);
            }
            telemetry.update();
        }

        propDetector.stopDetecting();

        telemetry.clearAll();
        telemetry.setAutoClear(false);
        telemetry.addData("Running", "%s %s,  prop location = %s", alliance, side, propLocation);
        if(side == Side.FRONT)
            telemetry.addData("Go through stage door", goThroughStageDoor);
        telemetry.addData("Place on backdrop", placeOnBackdrop);
        if(placeOnBackdrop)
            telemetry.addData("Use april tags", useAprilTags);
        status = telemetry.addData("\nStatus", "loading...");
        telemetry.update();

        // trajectories
        Pose2d startPose = null, backdropPose = null, stackPose = null, parkPose = null;
        Vector2d truss = null;
        TrajectorySequence spikeMarkTraj = null, toAprilTagDetection = null, toBackdrop, parkTraj = null;
        double spikeMarkBackDistance = 1.5, backdropForwardDistance = 3+.3, backdropBackDistance = 4;
        double approachSpeed = 2, moveAwaySpeed = 1;

        double backdropX = 48.5, backdropY = 0;
        switch (alliance) {
            // blue common TODO
            case BLUE:
                switch(propLocation) { // blue backdrop
                    case LEFT:
//                        backdropX -= side == Side.BACK ? 1.8-.4 : 1.8;
                        backdropY = 42;
                        break;
                    case CENTER:
//                        backdropX -= .7-.5;
                        backdropY = 39;
                        break;
                    case RIGHT:
//                        backdropX -= 3.5;
                        backdropY = 34;
                }
                backdropPose = new Pose2d(backdropX - 9, backdropY - 2.5, Math.toRadians(10));
                stackPose = new Pose2d(-56, 23, 0);
                truss = goThroughStageDoor ? new Vector2d(-3, 10-.5) : new Vector2d(-3, 63);
                break;
            // red common TODO
            case RED:
                switch(propLocation) { // red backdrop
                    case LEFT:
//                        backdropX += 1;
                        backdropY = -28+6;
                        break;
                    case CENTER:
//                        backdropX += side == Side.BACK ? 2.7 : 3.1;
                        backdropY = -26+6;
                        break;
                    case RIGHT:
//                        backdropX += 2.7;
                        backdropY = -36+6;
                }
                backdropPose = new Pose2d(backdropX, backdropY, Math.toRadians(0));
                stackPose = new Pose2d(-56, -35, 0);
                truss = goThroughStageDoor ? new Vector2d(-6, -6) : new Vector2d(-6, -60);
        }

        switch(side) {
            case BACK:
                switch(alliance) {
                    // BLUE BACK TODO
                    case BLUE:
                        startPose = new Pose2d(12, 64, -Math.PI / 2);
                        parkPose = new Pose2d(55, 58.8+.1, 0);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(21.0, 40, -Math.PI / 2), -Math.PI / 2)
                                        .back(2.5)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(15, 32.7, -Math.PI / 2), -Math.PI / 2)
                                        .back(4.5)
                                        .build();
                                break;
                            case RIGHT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .forward(10)
                                        .splineToSplineHeading(new Pose2d(7, 37, Math.PI), Math.PI)
                                        .back(2)
                                        .build();
                                parkPose = parkPose.plus(new Pose2d(0, 6+3, 0));
                        }
                        break;
                    // RED BACK TODO
                    case RED:
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

                // back traj todo
                toAprilTagDetection = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                        .addTemporalMarker(() -> robot.outtake.rotator.rotateFully())
                        .addTemporalMarker(.5, () -> robot.outtake.goToUpPosition())
                        .back(spikeMarkBackDistance)
                        .splineToSplineHeading(backdropPose, backdropPose.getHeading())
                        .build();
                if(placeOnBackdrop) {
                    parkTraj = robot.drive.trajectorySequenceBuilder(toAprilTagDetection.end())
                            .waitSeconds(1)
                            .back(backdropBackDistance, SampleMecanumDrive.getVelocityConstraint(moveAwaySpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addTemporalMarker(1.5, () -> robot.outtake.goToDownPosition())
                            .addTemporalMarker(2.2, () -> robot.outtake.rotator.retractFully())
                            .addTemporalMarker(2.5, () -> robot.outtake.stop())
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
                    // blue front TODO
                    case BLUE:
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
                    // red front todo
                    case RED:
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

                // front traj todo
                waitTime = placeOnBackdrop ? 8-2 - waitTime : 0;
                if(alliance == Alliance.BLUE && propLocation == Location.CENTER && goThroughStageDoor)
                    toAprilTagDetection = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                            .back(spikeMarkBackDistance)
                            .splineToConstantHeading(truss.plus(new Vector2d(-46, 3)), -Math.PI/2)
                            .splineToConstantHeading(truss.plus(new Vector2d(-33, 0)), 0)
                            .waitSeconds(waitTime)
                            .lineToConstantHeading(truss.plus(new Vector2d(12+21, 0)))
                            .addDisplacementMarker(() -> robot.outtake.rotator.rotateFully())
                            .UNSTABLE_addTemporalMarkerOffset(.5, () -> robot.outtake.goToPosition(Outtake.POSITION_2))
                            .splineToSplineHeading(backdropPose, backdropPose.getHeading())
                            .build();
                else
                    toAprilTagDetection = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                            .back(spikeMarkBackDistance)
                            .lineToConstantHeading(truss.plus(new Vector2d(-35, 0)))
                            .waitSeconds(waitTime)
                            .lineToConstantHeading(truss.plus(new Vector2d(33, 0)))
                            .addDisplacementMarker(() -> robot.outtake.rotator.rotateFully())
                            .UNSTABLE_addTemporalMarkerOffset(.5, () -> robot.outtake.goToPosition(Outtake.POSITION_2))
                            .splineToSplineHeading(backdropPose, backdropPose.getHeading())
                            .build();
                if(placeOnBackdrop) {
                    parkTraj = robot.drive.trajectorySequenceBuilder(toAprilTagDetection.end())
                            .waitSeconds(1)
                            .back(backdropBackDistance, SampleMecanumDrive.getVelocityConstraint(moveAwaySpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .addDisplacementMarker(() -> robot.outtake.rotator.retractFully())
                            .addTemporalMarker(2, () -> {
                                robot.outtake.goToDownPosition();
                                executorService.schedule(robot.outtake::stop, 1000, TimeUnit.MILLISECONDS);
                            })
                            .forward(1, SampleMecanumDrive.getVelocityConstraint(moveAwaySpeed * 3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
        robot.autoClaw.outAndIn();
        timer.reset();
        while(timer.seconds() < .5 && opModeIsActive());

        if(placeOnBackdrop) {
            status.setValue("moving to april tag detection at %.0f seconds", getRuntime() + (side == Side.FRONT ? waitTime : 0));
            telemetry.update();
            robot.drive.followTrajectorySequence(toAprilTagDetection);

            startPose = robot.drive.getPoseEstimate();
            double x = backdropX - startPose.getX(), y = backdropY - startPose.getY();
            AprilTagPose tagPose;
            if(useAprilTags) {
                // april tag detection TODO
                initializeAprilTagDetection();

                backdrop = AprilTagIDs.getBackdrop(alliance);
                int idOfInterest = backdrop.getId(propLocation);
                Telemetry.Item results = telemetry.addData("Results", null);
                Telemetry.Item movement = telemetry.addData("Movement", null);

                double aprilTagDetectionTime = 12-3 - waitTime, timeLeft = aprilTagDetectionTime, timeTo1stDetection = 0;
                boolean detectedSomething = false;
                timer.reset();
                while (timeLeft > 0 && opModeIsActive() && !gamepad.a) {
                    status.setValue("looking for %s %s (id %d)...%.1f s", alliance, propLocation, idOfInterest, timeLeft);
                    tagPose = getAprilTagPose(results, idOfInterest);
                    if(!detectedSomething && tagPose != null)
                        timeTo1stDetection = timer.seconds();
                    if(!detectedSomething && timeTo1stDetection != 0) {
                        telemetry.addData("Time to 1st detection", "%.1f sec", timeTo1stDetection);
                        detectedSomething = true;
                    }
                    if(tagPose != null) {
                        x = tagPose.z * INCHES_PER_METER - 1.7;
                        y = -tagPose.x * INCHES_PER_METER - 4;
                        movement.setValue("x = %.1f forward, y = %.1f %s", x, Math.abs(y), y > 0 ? "left" : "right");
                    }
                    telemetry.update();
                    timeLeft = aprilTagDetectionTime - timer.seconds();
                }
                if(!opModeIsActive())
                    requestOpModeStop();
                telemetry.removeItem(results);
            }
            toBackdrop = robot.drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() -> {
                        robot.outtake.rotator.rotateFully();
                        robot.outtake.goToPosition(side == Side.BACK ? Outtake.POSITION_1 : Outtake.POSITION_2);
                    })
                    .splineToLinearHeading(new Pose2d(startPose.getX() + x, startPose.getY() + y, 0), 0)
                    .forward(backdropForwardDistance, SampleMecanumDrive.getVelocityConstraint(approachSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            status.setValue("moving to backdrop");
            telemetry.update();
            robot.drive.followTrajectorySequence(toBackdrop);

            status.setValue("placing pixel");
            telemetry.update();
            timer.reset();
            while(timer.seconds() < 1 && opModeIsActive());
            robot.claw1.up();
            robot.claw2.up();
            while(timer.seconds() < 2 && opModeIsActive());
            robot.outtake.goToPosition(robot.outtake.getPosition() + 350+50, .1);
            while(!robot.outtake.isIdle() && opModeIsActive());
        }

        status.setValue("parking");
        telemetry.update();
        robot.drive.followTrajectorySequence(parkTraj);

        status.setValue("parked in %.0f seconds", getRuntime());
        telemetry.update();

        TeleOp.setStartPose(robot.drive.getPoseEstimate());

        while(opModeIsActive()) {
            if(gamepad.b && !gamepad.start)
                robot.drive.followTrajectorySequence(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                        .back(1)
                        .splineToSplineHeading(startPose, -startPose.getHeading())
                        .build()
                );
        }
        requestOpModeStop();
    }
    private void initializeAprilTagDetection() {
        try {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            double tagsize = 0.166, fx = 578.272, fy = 578.272, cx = 402.145, cy = 221.506;
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("Camera initialization failed", errorCode);
                    telemetry.update();
                }
            });
        } catch(Exception e) {
            telemetry.addData("Camera initialization failed", e);
            telemetry.update();
        }
    }
    private AprilTagPose getAprilTagPose(Telemetry.Item results, int idOfInterest) {
        currentAprilTagDetections = aprilTagDetectionPipeline.getLatestDetections();
        detectedTags = new ArrayList<>();
        if(currentAprilTagDetections.size() == 0)
            results.setValue("can't see any tags");
        else {
            for(AprilTagDetection tag : currentAprilTagDetections) {
                detectedTags.add(backdrop.getLocation(tag.id));
                if (tag.id == idOfInterest)
                    tagOfInterest = tag;
            }
            if(tagOfInterest != null)
                results.setValue("can see tag at: %s", aprilTagPoseToString(tagOfInterest.pose));
            else
                results.setValue("can see tags %s but not correct one", detectedTags);
        }
        return tagOfInterest == null ? null : tagOfInterest.pose;
    }
    @SuppressLint("DefaultLocale")
    private String aprilTagPoseToString(AprilTagPose pose) {
        return String.format("(%.02f, %.02f, %.02f)", pose.x, pose.y, pose.z);
    }
}