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
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.MecanumDrive;
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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto", group = "auto", preselectTeleOp = "TeleOp")
public class Auto extends LinearOpMode
{
    public enum ParkPosition {
        CORNER, CENTER
    }
    private static Alliance alliance = Alliance.BLUE;
    private static Side side = Side.FAR;
    private static boolean wait = false, goThroughStageDoor = true, placeOnBackdrop = false, useAprilTags = true, pickFromStack = true;
    private static ParkPosition parkPosition = ParkPosition.CENTER;
    private static boolean debugMode = true;

    private Location propLocation = Location.LEFT;

    private Robot robot;

    private boolean aprilTagDetectionInitialized = false;
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private static final double INCHES_PER_METER = 13.12336;
    private Backdrop backdrop;
    private AprilTagDetection tagOfInterest = null;
    private List<AprilTagDetection> currentAprilTagDetections;
    private List<Location> detectedTags;

    private ElapsedTime timer;
    private ScheduledExecutorService executorService;
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        TensorFlowObjectDetector propDetector = new TensorFlowObjectDetector(hardwareMap);
        executorService = Executors.newSingleThreadScheduledExecutor();
        timer = new ElapsedTime();
        boolean initialized = false,  propLocationOverride = false;
        double minInitTime = 5.5, waitTime = 15;

        Telemetry.Item status = telemetry.addData("Status", null).setRetained(true);
        telemetry.addLine().addData("Runtime", "%.0f", this::getRuntime).addData("loop time", "%.1f ms", timer::milliseconds);
        telemetry.addData("Debug mode (start + X/Y)", () -> debugMode + (debugMode ? " (turn off timer)" : ""));
        telemetry.addData("\nAlliance", () -> alliance);
        telemetry.addData("Side", () -> side);
        telemetry.addData("Wait (RB/LB)", () -> wait);
        telemetry.addData("Go through STAGE DOOR (LS)", () -> (side == Side.FAR || pickFromStack) ? goThroughStageDoor : "n/a");
        telemetry.addData("Place on BACKDROP (RT/LT)", () -> placeOnBackdrop);
        telemetry.addData("Pick from STACK (back + RT/LT)", () -> pickFromStack);
        telemetry.addData("Use APRIL TAGS (back + RB/LB)", () -> (placeOnBackdrop || pickFromStack) ? useAprilTags : "n/a");
        telemetry.addData("Park (RS)", () -> ((placeOnBackdrop && (wait || pickFromStack)) ? "n/a" : parkPosition));
        Telemetry.Item propLocationTelemetry = telemetry.addData("\nProp location", null).setRetained(true);

        // init loop todo
        while(opModeInInit() && !(gamepad1.start && gamepad1.back)) {
            if(!initialized) {
                double runtime = getRuntime();
                if (runtime < minInitTime)
                    status.setValue("initializing please wait...%.1f", minInitTime - runtime);
                else {
                    initialized = true;
                    status.setValue("initialized");
                }
            }

            timer.reset();

            // gamepad input
            {
                // ALLIANCE
                if(gamepad1.x && !gamepad1.start)
                    alliance = Alliance.BLUE;
                else if(gamepad1.b && !gamepad1.start)
                    alliance = Alliance.RED;

                // SIDE
                if(gamepad1.a && !gamepad1.start) {
                    side = Side.NEAR;
                    wait = false;
                    parkPosition = ParkPosition.CORNER;
                } else if(gamepad1.y && !gamepad1.start) {
                    side = Side.FAR;
                    wait = true;
                }

                // WAIT
                if(gamepad1.right_bumper && !gamepad1.back)
                    wait = true;
                else if(gamepad1.left_bumper && !gamepad1.back)
                    wait = false;

                // GO THROUGH STAGE DOOR
                if(gamepad1.left_stick_y < -.5) {
                    goThroughStageDoor = true;
                    if(!placeOnBackdrop)
                        parkPosition = ParkPosition.CENTER;
                } else if(gamepad1.left_stick_y > .5) {
                    goThroughStageDoor = false;
                    if(!placeOnBackdrop)
                        parkPosition = ParkPosition.CORNER;
                }

                // PLACE ON BACKDROP
                if (gamepad1.right_trigger > .1 && !gamepad1.back)
                    placeOnBackdrop = true;
                else if(gamepad1.left_trigger > .1 && !gamepad1.back) {
                    placeOnBackdrop = false;
                    parkPosition = (side == Side.NEAR ? ParkPosition.CORNER : ParkPosition.CENTER);
                }

                // PICK FROM STACK
                if(gamepad1.right_trigger > .1 && gamepad1.back) {
                    pickFromStack = true;
                    wait = false;
                    goThroughStageDoor = true;
                } else if(gamepad1.left_trigger > .1 && gamepad1.back) {
                    pickFromStack = false;
                    if(side == Side.NEAR) {
                        wait = false;
                        parkPosition = ParkPosition.CORNER;
                    } else wait = true;
                }

                // APRIL TAGS
                if(placeOnBackdrop || pickFromStack) {
                    if(gamepad1.right_bumper && gamepad1.back)
                        useAprilTags = true;
                    else if(gamepad1.left_bumper && gamepad1.back)
                        useAprilTags = false;
                }

                // PARK
                if(gamepad1.right_stick_y < -.1)
                    parkPosition = ParkPosition.CENTER;
                else if(gamepad1.right_stick_y > .1)
                    parkPosition = ParkPosition.CORNER;

                // DEBUG MODE
                if(gamepad1.start && gamepad1.x)
                    debugMode = true;
                else if(gamepad1.start && gamepad1.y)
                    debugMode = false;

                // PROP LOCATION OVERRIDE
                if (gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right) {
                    propLocationOverride = true;
                    if (gamepad1.dpad_left)
                        propLocation = Location.LEFT;
                    else if (gamepad1.dpad_up)
                        propLocation = Location.CENTER;
                    else
                        propLocation = Location.RIGHT;
                    propLocationTelemetry.setValue(propLocation + " (override)");
                } else if (gamepad1.back)
                    propLocationOverride = false;
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

        double beginningWaitTime = Math.max(minInitTime - getRuntime(), (side == Side.NEAR && wait ? waitTime : 0));
        resetRuntime();
        telemetry.clearAll();
        status.setValue("loading...");
        if(propLocationOverride) {
            status.setValue("waiting...%.1f", () -> beginningWaitTime - getRuntime());
            telemetry.addData("Prop location", propLocation).setRetained(true);
            while(getRuntime() < beginningWaitTime && opModeIsActive())
                telemetry.update();
        } else {
            status.setValue("detecting prop location...%.1f", () -> beginningWaitTime - getRuntime());
            telemetry.addData("Prop location", () -> propLocation);
            while(getRuntime() < beginningWaitTime && opModeIsActive()) {
                propDetector.update();
                propLocation = propDetector.getLocation();
                telemetry.addLine();
                propDetector.telemetryAll(telemetry);
                telemetry.update();
            }
        }

        propDetector.stopDetecting();

        robot.claw2.down();
        if(pickFromStack)
            robot.claw1.up();
        else robot.claw1.down();

        telemetry.clearAll();
        telemetry.setAutoClear(false);
        telemetry.setMsTransmissionInterval(500);
        telemetry.addData("Running", "%s %s,  prop location = %s", alliance, side, propLocation);
        if(wait)
            telemetry.addLine("WAIT");
        if(side == Side.FAR || pickFromStack)
            telemetry.addLine("Go through " + (goThroughStageDoor ? "stage door" : "truss by wall"));
        telemetry.addLine((placeOnBackdrop ? "Place" : "Don't place") + " on backdrop");
        if(pickFromStack)
            telemetry.addLine("Pick from stack");
        if(placeOnBackdrop || pickFromStack)
            telemetry.addLine((useAprilTags ? "Use" : "Don't use") + " april tags");
        if((!wait && !pickFromStack) || !placeOnBackdrop)
            telemetry.addData("Park in", parkPosition);
        if(debugMode)
            telemetry.addLine("DEBUG");
        status = telemetry.addData("\nStatus", "loading...");
        telemetry.update();

        // trajectories todo
        Pose2d startPose = null, backdropPose, stackPose = null, parkPose = null;
        Pose2d aprilTagOffset = new Pose2d(-5-2, 0, 0);
        Vector2d trussFront = null, trussBack;
        TrajectorySequence spikeMarkTraj = null, toAprilTagDetection = null, toBackdrop = null, parkTraj = null;
        TrajectorySequence frontToStack = null, frontToWait = null;
        TrajectorySequence backdropToStack, stackToBackdrop;
        double spikeMarkBackDistance = 1.5, backdropForwardDistance = 4.6;

        double trussFrontX = -38;
        double backdropX = 49, backdropY = 0;
        switch (alliance) {
            // blue common todo
            case BLUE:
                backdropY += 2;
//                if(side == Side.FAR) {
//                    backdropX += 2.5;
//                    backdropY -= 1;
//                }
                switch(propLocation) { // blue backdrop
                    case LEFT:
//                        backdropX -= side == Side.BACK ? 1.8-.4 : 1.8;
                        backdropY += 42;
                        break;
                    case CENTER:
//                        backdropX -= .7-.5;
                        backdropY += 36.5;
                        break;
                    case RIGHT:
//                        backdropX -= 3.5;
                        backdropY += 35;
                }
                stackPose = new Pose2d(-57, 6, 0);
                trussFront = new Vector2d(trussFrontX, goThroughStageDoor ? 9.5 : 63);
                parkPose = parkPosition == ParkPosition.CORNER ? new Pose2d(66, 63, 0) : new Pose2d(62+2, 9-1, 0);
                break;
            // red common todo
            case RED:
                backdropX -= 5;
                backdropY += -25;
//                if(side == Side.FAR) {
//                    backdropX += 3;
//                    backdropY -= 4;
//                }
                switch(propLocation) { // red backdrop
                    case LEFT:
//                        backdropX += 1;
                        backdropY += 3;
                        break;
                    case CENTER:
//                        backdropX += side == Side.BACK ? 2.7 : 3.1;
//                        backdropY = -26;
                        break;
                    case RIGHT:
//                        backdropX += 2.7;
                        backdropY -= 8;
                }
                stackPose = new Pose2d(-56, -35, 0);
                trussFront = new Vector2d(trussFrontX, goThroughStageDoor ? -6 : -60);
                parkPose = parkPosition == ParkPosition.CORNER ? new Pose2d(47, -59, 0) : new Pose2d(52, -10, 0);
        }
        backdropPose = new Pose2d(backdropX, backdropY, 0);
        Trajectory approachBackdrop = robot.drive.trajectoryBuilder(backdropPose)
                .forward(backdropForwardDistance, MecanumDrive.getVelConstraint(2+3), MecanumDrive.getAccelConstraint())
                .build();
        Trajectory leaveBackdrop = robot.drive.trajectoryBuilder(backdropPose.plus(new Pose2d(backdropForwardDistance, 0)))
                .back(backdropForwardDistance, MecanumDrive.getVelConstraint(8), MecanumDrive.getAccelConstraint())
                .build();
        trussBack = trussFront.plus(new Vector2d(33, 0));
        backdropToStack = robot.drive.trajectorySequenceBuilder(backdropPose.plus(new Pose2d(backdropForwardDistance)))
                .addTrajectory(leaveBackdrop)
                .splineTo(trussBack, Math.PI/2 * (alliance == Alliance.BLUE ? -1 : 1))
                .splineTo(trussFront, Math.PI)
                .splineToSplineHeading(stackPose, Math.PI)
                .build();
        stackToBackdrop = robot.drive.trajectorySequenceBuilder(stackPose)
                .splineTo(trussFront, 0)
                .splineTo(trussBack, 0)
                .addDisplacementMarker(() -> robot.outtake.rotator.rotateFully())
                .UNSTABLE_addTemporalMarkerOffset(.4, () -> robot.outtake.goToPosition(Outtake.POSITIONS[1]))
                .splineToSplineHeading(useAprilTags ? backdropPose.plus(aprilTagOffset) : backdropPose, Math.PI/2 * (alliance == Alliance.BLUE ? 1 : -1))
                .addTrajectory(approachBackdrop)
                .build();

        switch(side) {
            case NEAR:
                switch(alliance) {
                    // blue back todo
                    case BLUE:
                        startPose = new Pose2d(12, 64, -Math.PI / 2);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineTo(new Vector2d(24, 40), -Math.PI / 2)
                                        .back(3+.5)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineTo(new Vector2d(12, 32.7), -Math.PI / 2)
                                        .back(4.5)
                                        .build();
                                break;
                            case RIGHT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .forward(10)
                                        .splineToSplineHeading(new Pose2d(7, 37, Math.PI), Math.PI)
                                        .back(2)
                                        .build();
                        }
                        break;
                    // red back todo
                    case RED:
                        startPose = new Pose2d(12, -61, Math.PI / 2);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .forward(13)
                                        .splineToSplineHeading(new Pose2d(7.5, -33, Math.PI), Math.PI)
                                        .back(2)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(12, -30, Math.PI / 2), Math.PI / 2)
                                        .back(2+1)
                                        .build();
                                break;
                            case RIGHT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(23, -34, Math.PI / 2), Math.PI / 2)
                                        .back(4)
                                        .build();
                        }
                }

                // back traj todo
                if(placeOnBackdrop) {
                    if(useAprilTags)
                        toAprilTagDetection = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                                .addTemporalMarker(() -> robot.outtake.rotator.rotateFully())
                                .addTemporalMarker(.5, () -> robot.outtake.goToUpPosition())
                                .back(spikeMarkBackDistance)
                                .splineToSplineHeading(backdropPose.plus(aprilTagOffset), 0)
                                .build();
                    else
                        toBackdrop = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                                .addTemporalMarker(.5, () -> robot.outtake.rotator.rotateFully())
                                .addTemporalMarker(1, () -> robot.outtake.goToPosition(side == Side.NEAR ? Outtake.POSITIONS[0] : Outtake.POSITIONS[1]))
                                .back(spikeMarkBackDistance)
                                .splineToSplineHeading(backdropPose, 0)
                                .addTrajectory(approachBackdrop)
                                .build();
                } else {
                    parkTraj = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                            .back(spikeMarkBackDistance)
                            .splineToSplineHeading(parkPose, (alliance == Alliance.BLUE ? 1 : -1) * Math.PI/6)
                            .build();
                }
                break;
            case FAR:
                switch(alliance) {
                    // blue front todo
                    case BLUE:
                        startPose = new Pose2d(-36, 64, -Math.PI / 2);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .forward(7)
                                        .splineToSplineHeading(new Pose2d(-31, 34+1, 0), 0)
                                        .back(3)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-40, 24.5+1, 0), -Math.PI/2)
                                        .back(3)
                                        .build();
//                                spikeMarkBackDistance += 3;
                                break;
                            case RIGHT:
//                                telemetry.log().add("start pose should be (%.0f, %.0f)", startPose.getX(), startPose.getY());
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToLinearHeading(new Pose2d(-55, 33, 0), -Math.PI/2)
                                        .back(3)
                                        .build();
                                spikeMarkBackDistance = 0.1;
                                break;
                        }
                        break;
                    // red front todo
                    case RED:
                        startPose = new Pose2d(-36, -61, Math.PI / 2);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-42.5-.3,~ -31, Math.PI), Math.PI / 2)
                                        .back(4)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-40, -24-.8, 0), Math.PI / 2)
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
                if(pickFromStack) {
                    if(propLocation == Location.RIGHT)
                        frontToStack = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                                .strafeRight(12)
                                .splineToConstantHeading(stackPose.vec(), -Math.PI/2)
                                .build();
                    else
                        frontToStack = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                                .back(spikeMarkBackDistance)
                                .splineToConstantHeading(stackPose.vec(), Math.PI)
                                .build();
                    frontToWait = robot.drive.trajectorySequenceBuilder(stackPose.plus(new Pose2d(5+3, 15-7 * (alliance == Alliance.BLUE ? 1 : -1))))
                            .splineTo(trussFront, 0)
                            .build();
                } else {
                    if (goThroughStageDoor && (propLocation == Location.CENTER || propLocation == (alliance == Alliance.BLUE ? Location.RIGHT : Location.LEFT)))
                        frontToWait = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                                .back(spikeMarkBackDistance)
                                .splineToConstantHeading(trussFront.plus(new Vector2d(-10, 3 * (alliance == Alliance.BLUE ? 1 : -1))), Math.PI/2 * (alliance == Alliance.BLUE ? -1 : 1))
                                .splineToConstantHeading(trussFront, 0)
                                .build();
                    else
                        frontToWait = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                                .back(spikeMarkBackDistance)
                                .lineToConstantHeading(trussFront)
                                .build();
                }
                if(placeOnBackdrop) {
                    if(useAprilTags)
                        toAprilTagDetection = robot.drive.trajectorySequenceBuilder(frontToWait.end())
                                .lineToConstantHeading(trussBack)
                                .addDisplacementMarker(() -> robot.outtake.rotator.rotateFully())
                                .UNSTABLE_addTemporalMarkerOffset(.5, () -> robot.outtake.goToPosition(Outtake.POSITIONS[1]))
                                .splineToSplineHeading(backdropPose.plus(aprilTagOffset), Math.PI / 2 * (alliance == Alliance.BLUE ? 1 : -1))
                                .build();
                    else
                        toBackdrop = robot.drive.trajectorySequenceBuilder(frontToWait.end())
                                .lineToConstantHeading(trussBack)
                                .addDisplacementMarker(() -> robot.outtake.rotator.rotateFully())
                                .UNSTABLE_addTemporalMarkerOffset(.5, () -> robot.outtake.goToPosition(Outtake.POSITIONS[1]))
                                .splineToSplineHeading(backdropPose, Math.PI/2 * (alliance == Alliance.BLUE ? 1 : -1))
                                .addTrajectory(approachBackdrop)
                                .build();
                } else
                    parkTraj = robot.drive.trajectorySequenceBuilder(frontToWait.end())
                            .lineToConstantHeading(trussBack)
                            .splineToSplineHeading(parkPose, 0)
                            .build();
        }

        // start of movement todo
//        if(alliance == Alliance.BLUE && side == Side.FAR)
//            startPose = new Pose2d(-36, 64, -Math.PI / 2);
//        telemetry.log().add("start pose is (%.0f, %.0f)", startPose.getX(), startPose.getY());
        robot.drive.setPoseEstimate(startPose);

        status.setValue("moving to spike mark");
        telemetry.update();
        robot.drive.followTrajectorySequence(spikeMarkTraj);

        status.setValue("releasing pixel");
        telemetry.update();
        robot.autoClaw.outAndIn();
        timer.reset();
        while(timer.seconds() < .5 && opModeIsActive());
        if(debugMode)
            pause(status);

        if (side == Side.FAR) {
            if(pickFromStack) {
                robot.drive.followTrajectorySequence(frontToStack);
                if(debugMode)
                    pause(status);
                pickFromStack();
            }
//            robot.drive.followTrajectorySequence(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
//                    .lineTo(frontToWait.start().vec())
//                    .build());
            robot.drive.followTrajectorySequence(frontToWait);
            if(wait) {
                status.setValue("waiting...%.1f", () -> waitTime - getRuntime());
                while(getRuntime() < waitTime && opModeIsActive()) {
                    telemetry.update();
                }
            } else if(debugMode) pause(status);
        }

        if(placeOnBackdrop) {
            if(useAprilTags) {
                // april tag detection todo
                status.setValue("moving to april tag detection at %.0f seconds", getRuntime());
                telemetry.update();
                robot.drive.followTrajectorySequenceAsync(toAprilTagDetection);

                if(!aprilTagDetectionInitialized)
                    initializeAprilTagDetection();

                backdrop = AprilTagIDs.getBackdrop(alliance);
                int idOfInterest = backdrop.getId(propLocation);
                Telemetry.Item results = telemetry.addData("\nResults", null);

                assert toAprilTagDetection != null;
                startPose = toAprilTagDetection.end();
                double x = -aprilTagOffset.getX(), y = -aprilTagOffset.getY();
                AprilTagPose tagPose = null;

                double maxAprilTagDetectionTime = wait ? 6 : 8;
                double timeLeft = maxAprilTagDetectionTime;
                timer.reset();
                while (((tagPose == null && timeLeft > 0) || robot.drive.isBusy()) && opModeIsActive()) {
                    robot.drive.update();
                    status.setValue("looking for %s %s (id %d)...%.1f s", alliance, propLocation, idOfInterest, timeLeft);
                    tagPose = getAprilTagPose(idOfInterest, results);
                    telemetry.update();
                    timeLeft = maxAprilTagDetectionTime - timer.seconds();
                }
                if(tagPose != null) {
                    telemetry.addData("(time to detection", "%.1f sec)", timer.seconds());
                    x = tagPose.z * INCHES_PER_METER - 3.5;
                    y = -tagPose.x * INCHES_PER_METER - 5;
                }
                telemetry.addData("Moving","x = %.1f forward, y = %.1f %s", x, Math.abs(y), y > 0 ? "left" : "right");
                telemetry.update();
                toBackdrop = robot.drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(() -> robot.outtake.rotator.rotateFully())
                        .addTemporalMarker(.5, () -> robot.outtake.goToPosition(side == Side.NEAR ? Outtake.POSITIONS[0] : Outtake.POSITIONS[1]))
                        .splineToLinearHeading(new Pose2d(startPose.getX() + x, startPose.getY() + y, 0), 0)
                        .addTrajectory(approachBackdrop)
                        .build();
            }

            status.setValue("moving to backdrop");
            telemetry.update();
            robot.drive.followTrajectorySequence(toBackdrop);

            status.setValue("placing pixel");
            telemetry.update();
            placeOnBackdrop();

            if(side == Side.NEAR && pickFromStack) {
                robot.drive.followTrajectorySequence(backdropToStack);
                if(debugMode)
                    pause(status);
                pickFromStack();
                robot.drive.followTrajectorySequence(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                        .splineToSplineHeading(stackToBackdrop.start(), 0)
                        .build());
                if(debugMode)
                    pause(status);
                robot.drive.followTrajectorySequence(stackToBackdrop);
                placeOnBackdrop();
            }

            if(wait || pickFromStack)
                parkTraj = robot.drive.trajectorySequenceBuilder(toBackdrop.end())
                        .waitSeconds(1)
                        .addTrajectory(leaveBackdrop)
                        .addDisplacementMarker(() -> {
                            robot.outtake.goToDownPosition();
                            executorService.schedule(robot.outtake.rotator::retractFully, 700, TimeUnit.MILLISECONDS);
                            executorService.schedule(robot.outtake::stop, 800, TimeUnit.MILLISECONDS);
                        })
                        .forward(1, MecanumDrive.getVelConstraint(10), MecanumDrive.getAccelConstraint())
                        .build();
            else
                parkTraj = robot.drive.trajectorySequenceBuilder(toBackdrop.end())
//                    .waitSeconds(1-.5)
                    .back(backdropForwardDistance, MecanumDrive.getVelConstraint(10), MecanumDrive.getAccelConstraint())
                    .addTemporalMarker(1.5, () -> robot.outtake.goToDownPosition())
                    .addTemporalMarker(1.8, () -> robot.outtake.rotator.retractFully())
                    .addTemporalMarker(2.1, () -> robot.outtake.stop())
                    .splineToConstantHeading(parkPose.vec(), 0)
                    .build();
        }

        status.setValue("parking");
        telemetry.update();
        robot.drive.followTrajectorySequence(parkTraj);

        status.setValue("parked in %.0f seconds", getRuntime());
        telemetry.update();

        TeleOp.setStartPose(robot.drive.getPoseEstimate());

        timer.reset();
        while (timer.seconds() < 2 && opModeIsActive());
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
            aprilTagDetectionInitialized = true;
        } catch(Exception e) {
            telemetry.addData("Camera initialization failed", e);
            telemetry.update();
        }
    }
    private AprilTagPose getAprilTagPose(int idOfInterest, Telemetry.Item results) {
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

    private void pickFromStack() {
        robot.outtake.rotator.retractFully();
        robot.intake.lower();
        robot.intake.in();
        robot.drive.followTrajectorySequence(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .strafeLeft(10, MecanumDrive.getVelConstraint(.2* DriveConstants.MAX_VEL), MecanumDrive.getAccelConstraint())
                .build());
        timer.reset();
        while(timer.seconds() < 1 && opModeIsActive());
        robot.intake.raise();
        robot.drive.followTrajectorySequence(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .forward(10)
                .build());
        executorService.schedule(() -> {
            robot.intake.stop();
            robot.claw1.down();
        }, 3, TimeUnit.SECONDS);
    }
    private void placeOnBackdrop() {
        timer.reset();
        while(timer.seconds() < 0.6/2 && opModeIsActive());
        robot.claw1.up();
        robot.claw2.up();
//        while(timer.seconds() < 2 && opModeIsActive());
        robot.outtake.goToPosition(robot.outtake.getPosition() + 350+50, .1*3);
        while(!robot.outtake.isIdle() && opModeIsActive());
    }

    private void pause(Telemetry.Item status) {
        status.setValue("paused, press any button to continue");
        telemetry.update();
        while(gamepad1.atRest() && opModeIsActive());
        status.setValue("resuming");
        telemetry.update();
    }
}