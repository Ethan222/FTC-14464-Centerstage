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
    public enum ParkPosition {
        CORNER, CENTER
    }
    private static Alliance alliance = Alliance.BLUE;
    private static Side side = Side.NEAR;
    private static boolean wait = false, goThroughStageDoor = true, placeOnBackdrop = true, useAprilTags = true, pickFromStack = false;
    private static ParkPosition parkPosition = ParkPosition.CORNER;

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
        boolean initialized = false;
        double minInitTime = 5.5;
        boolean propLocationOverride = false;

//        telemetry.setMsTransmissionInterval(150);
//        telemetry.setAutoClear(true);

        Telemetry.Item status = telemetry.addData("Status", null).setRetained(true);
        telemetry.addLine().addData("Runtime", "%.0f", this::getRuntime).addData("loop time", "%.2f ms", timer::milliseconds).setRetained(true);
        telemetry.addData("\nAlliance", () -> alliance).setRetained(true);
        telemetry.addData("Side", () -> side).setRetained(true);
        telemetry.addData("Wait (RB/LB)", () -> Boolean.toString(wait).toUpperCase()).setRetained(true);
        telemetry.addData("Go through stage door (LS)", () -> (side == Side.FAR || pickFromStack) ? Boolean.toString(goThroughStageDoor).toUpperCase() : "N/A").setRetained(true);
        telemetry.addData("Place on backdrop (RT/LT)", () -> Boolean.toString(placeOnBackdrop).toUpperCase()).setRetained(true);
        telemetry.addData("Pick from stack (back + RT/LT)", () -> Boolean.toString(pickFromStack).toUpperCase()).setRetained(true);
        telemetry.addData("Use april tags (back + RB/LB)", () -> (placeOnBackdrop || pickFromStack) ? Boolean.toString(useAprilTags).toUpperCase() : "N/A").setRetained(true);
        telemetry.addData("Park (RS)", () -> ((placeOnBackdrop && (wait || pickFromStack)) ? "N/A" : parkPosition));
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
                if(gamepad1.x)
                    alliance = Alliance.BLUE;
                else if(gamepad1.b && !gamepad1.start)
                    alliance = Alliance.RED;

                // SIDE
                if(gamepad1.y) {
                    side = Side.NEAR;
                    wait = false;
                    parkPosition = ParkPosition.CORNER;
                } else if(gamepad1.a && !gamepad1.start) {
                    side = Side.FAR;
                    wait = true;
                }

                // GO THROUGH STAGE DOOR
                if(gamepad1.left_stick_y < -.5)
                    goThroughStageDoor = true;
                else if(gamepad1.left_stick_y > .5)
                    goThroughStageDoor = false;

                // PLACE ON BACKDROP
                if (gamepad1.right_trigger > .1) placeOnBackdrop = true;
                else if(gamepad1.left_trigger > .1) {
                    placeOnBackdrop = false;
                    parkPosition = (side == Side.NEAR ? ParkPosition.CORNER : ParkPosition.CENTER);
                }

                // PICK FROM STACK
                if(gamepad1.right_trigger > .1 && gamepad1.back) {
                    pickFromStack = true;
                    wait = false;
                    goThroughStageDoor = true;
                    placeOnBackdrop = true;
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

        double waitTime = Math.max(minInitTime - getRuntime(), 0);
        resetRuntime();
        robot.claw1.down();
        robot.claw2.down();
        telemetry.clearAll();
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
        status = telemetry.addData("\nStatus", "loading...");
        telemetry.update();

        // trajectories todo
        Pose2d startPose = null, backdropPose = null, stackPose = null, parkPose = null;
        Vector2d truss = null;
        TrajectorySequence spikeMarkTraj = null, toAprilTagDetection = null, toBackdrop, parkTraj = null;
        TrajectorySequence backdropToStack, stackToBackdrop;
        double spikeMarkBackDistance = 1.5, backdropForwardDistance = 4.6, backdropBackDistance = 5;
        double approachSpeed = 2, moveAwaySpeed = 8;

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
                backdropPose = new Pose2d(backdropX - 5, backdropY - 0, 0);
                stackPose = new Pose2d(-56, 23, 0);
                truss = goThroughStageDoor ? new Vector2d(-3, 9.5) : new Vector2d(-3, 63);
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
                backdropPose = new Pose2d(backdropX - 3, backdropY, 0);
                stackPose = new Pose2d(-56, -35, 0);
                truss = goThroughStageDoor ? new Vector2d(-6, -6) : new Vector2d(-6, -60);
        }

        switch(side) {
            case NEAR:
                switch(alliance) {
                    // blue back todo
                    case BLUE:
                        startPose = new Pose2d(12, 64, -Math.PI / 2);
                        parkPose = new Pose2d(55, 59.9, 0);
                        switch (propLocation) {
                            case LEFT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(21.3, 40, -Math.PI / 2), -Math.PI / 2)
                                        .back(2.5)
                                        .build();
                                break;
                            case CENTER:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(12, 32.7, -Math.PI / 2), -Math.PI / 2)
                                        .back(4.5)
                                        .build();
                                break;
                            case RIGHT:
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .forward(10)
                                        .splineToSplineHeading(new Pose2d(7, 37, Math.PI), Math.PI)
                                        .back(2)
                                        .build();
//                                parkPose = parkPose.plus(new Pose2d(0, 5, 0));
                        }
                        break;
                    // red back todo
                    case RED:
                        startPose = new Pose2d(12, -61, Math.PI / 2);
                        parkPose = new Pose2d(47, -59, 0);
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
                toAprilTagDetection = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                        .addTemporalMarker(() -> robot.outtake.rotator.rotateFully())
                        .addTemporalMarker(.5, () -> robot.outtake.goToUpPosition())
                        .back(spikeMarkBackDistance)
                        .splineToSplineHeading(backdropPose, 0)
                        .build();
                if(!placeOnBackdrop) {
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
                        parkPose = goThroughStageDoor ? new Pose2d(46, 14, 0) : new Pose2d(52, 62, 0);
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
                                        .splineToSplineHeading(new Pose2d(-40, 23+1.5, 0), 0)
                                        .back(2+1)
                                        .build();
                                spikeMarkBackDistance += 3;
                                break;
                            case RIGHT:
                                telemetry.log().add("start pose should be (%.0f, %.0f)", startPose.getX(), startPose.getY());
                                spikeMarkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                                        .splineToSplineHeading(new Pose2d(-47, 40, -Math.PI/2), -Math.PI / 2)
                                        .back(3)
                                        .build();
                                break;
                        }
                        break;
                    // red front todo
                    case RED:
                        startPose = new Pose2d(-36, -61, Math.PI / 2);
                        parkPose = goThroughStageDoor ? new Pose2d(52, -10, 0) : new Pose2d(47, -59, 0);
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
                waitTime = placeOnBackdrop ? (useAprilTags ? 5+1 : 12) - waitTime : 0;
                if(alliance == Alliance.BLUE && propLocation == Location.CENTER && goThroughStageDoor)
                    toAprilTagDetection = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                            .back(spikeMarkBackDistance)
                            .splineToConstantHeading(truss.plus(new Vector2d(-46, 3)), -Math.PI/2)
                            .splineToConstantHeading(truss.plus(new Vector2d(-33, 0)), 0)
                            .waitSeconds(waitTime)
                            .lineToConstantHeading(truss.plus(new Vector2d(33, 0)))
                            .addDisplacementMarker(() -> robot.outtake.rotator.rotateFully())
                            .UNSTABLE_addTemporalMarkerOffset(.5, () -> robot.outtake.goToPosition(Outtake.POSITION_2))
                            .splineToSplineHeading(backdropPose, Math.PI/2 * (alliance == Alliance.BLUE ? 1 : -1))
                            .build();
                else if(alliance == Alliance.BLUE && propLocation == Location.RIGHT && goThroughStageDoor) {
                    startPose = spikeMarkTraj.end();
                    toAprilTagDetection = robot.drive.trajectorySequenceBuilder(startPose)
                            .back(.3)
                            .splineToConstantHeading(new Vector2d(startPose.getX() + 9+1, startPose.getY()+1), -Math.PI/3)
                            .splineToConstantHeading(truss.plus(new Vector2d(-30, 0)), -Math.PI/2)
                            .turn(Math.PI/2)
                            .waitSeconds(waitTime)
                            .lineToConstantHeading(truss.plus(new Vector2d(33, 0)))
                            .addDisplacementMarker(() -> robot.outtake.rotator.rotateFully())
                            .UNSTABLE_addTemporalMarkerOffset(.5, () -> robot.outtake.goToPosition(Outtake.POSITION_2))
                            .splineToSplineHeading(backdropPose, Math.PI/2 * (alliance == Alliance.BLUE ? 1 : -1))
                            .build();
                } else
                    toAprilTagDetection = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                            .back(spikeMarkBackDistance)
                            .lineToConstantHeading(truss.plus(new Vector2d(-35, 0)))
                            .waitSeconds(waitTime)
                            .lineToConstantHeading(truss.plus(new Vector2d(33, 0)))
                            .addDisplacementMarker(() -> robot.outtake.rotator.rotateFully())
                            .UNSTABLE_addTemporalMarkerOffset(.5, () -> robot.outtake.goToPosition(Outtake.POSITION_2))
                            .splineToSplineHeading(backdropPose, Math.PI/2 * (alliance == Alliance.BLUE ? 1 : -1))
                            .build();
                if(!placeOnBackdrop)
                    parkTraj = robot.drive.trajectorySequenceBuilder(spikeMarkTraj.end())
                            .back(spikeMarkBackDistance)
                            .lineToConstantHeading(truss.plus(new Vector2d(-33, 0)))
                            .lineToConstantHeading(truss.plus(new Vector2d(12, 0)))
                            .splineToSplineHeading(parkPose, 0)
                            .build();
        }

        // start of movement todo
        if(alliance == Alliance.BLUE && side == Side.FAR)
            startPose = new Pose2d(-36, 64, -Math.PI / 2);
        telemetry.log().add("start pose is (%.0f, %.0f)", startPose.getX(), startPose.getY());
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
            status.setValue("moving to april tag detection at %.0f seconds", getRuntime() + (side == Side.FAR ? waitTime + 2 : 0));
            telemetry.update();
            robot.drive.followTrajectorySequence(toAprilTagDetection);

            startPose = robot.drive.getPoseEstimate();
            double x = backdropX - startPose.getX(), y = backdropY - startPose.getY();
            AprilTagPose tagPose = null;
            if(useAprilTags) {
                // april tag detection todo
                initializeAprilTagDetection();

                backdrop = AprilTagIDs.getBackdrop(alliance);
                int idOfInterest = backdrop.getId(propLocation);
                Telemetry.Item results = telemetry.addData("\nResults", null);

                double maxAprilTagDetectionTime = side == Side.NEAR ? 4.0 : 2.5+1;
                double timeLeft = maxAprilTagDetectionTime;
                timer.reset();
                while (tagPose == null && timeLeft > 0 && opModeIsActive()) {
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
            }
            telemetry.addData("Moving","x = %.1f forward, y = %.1f %s", x, Math.abs(y), y > 0 ? "left" : "right");
            telemetry.update();
            toBackdrop = robot.drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(() -> robot.outtake.rotator.rotateFully())
                    .addTemporalMarker(.5, () -> robot.outtake.goToPosition(side == Side.NEAR ? Outtake.POSITION_1 : Outtake.POSITION_2))
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

            if(side == Side.NEAR)
                parkTraj = robot.drive.trajectorySequenceBuilder(toBackdrop.end())
                    .waitSeconds(1)
                    .back(backdropBackDistance, SampleMecanumDrive.getVelocityConstraint(moveAwaySpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addTemporalMarker(1.5, () -> robot.outtake.goToDownPosition())
                    .addTemporalMarker(2.1, () -> robot.outtake.rotator.retractFully())
                    .addTemporalMarker(2.3, () -> robot.outtake.stop())
                    .splineToLinearHeading(parkPose, (alliance == Alliance.BLUE ? 1 : -1) * Math.PI/6)
                    .build();
            else
                parkTraj = robot.drive.trajectorySequenceBuilder(toBackdrop.end())
                        .waitSeconds(1)
                        .back(backdropBackDistance, SampleMecanumDrive.getVelocityConstraint(moveAwaySpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .addDisplacementMarker(() -> {
                            robot.outtake.goToDownPosition();
                            executorService.schedule(robot.outtake.rotator::retractFully, 700, TimeUnit.MILLISECONDS);
                            executorService.schedule(robot.outtake::stop, 800, TimeUnit.MILLISECONDS);
                        })
                        .forward(1, SampleMecanumDrive.getVelocityConstraint(moveAwaySpeed * 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
        }

        status.setValue("parking");
        telemetry.update();
        robot.drive.followTrajectorySequence(parkTraj);

        // todo
        if(!placeOnBackdrop) {
            robot.outtake.rotator.rotateFully();
            timer.reset();
            while(timer.seconds() < .8 && opModeIsActive());
            robot.claw1.up();
            robot.claw2.up();
            timer.reset();
            while(timer.seconds() < 1 && opModeIsActive());
            robot.autoClaw.outAndIn();
        }

        status.setValue("parked in %.0f seconds", getRuntime());
        telemetry.update();

        TeleOp.setStartPose(robot.drive.getPoseEstimate());

        timer.reset();
        while (timer.seconds() < 1 && opModeIsActive());
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
}