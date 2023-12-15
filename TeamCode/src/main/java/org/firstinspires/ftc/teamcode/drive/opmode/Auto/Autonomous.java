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
import org.firstinspires.ftc.teamcode.drive.opmode.Auto.OpenCv.TeamPropDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "auto")
public class Autonomous extends LinearOpMode
{
    private Alliance alliance = Alliance.BLUE;

    enum Side {
        LEFT, RIGHT
    }
    private Side side = Side.LEFT;

    Robot robot;
    TeamPropDetector teamPropDetector;

    private Location teamPropLocation = Location.CENTER;

    @Override
    public void runOpMode()
    {
        telemetry.addLine("Initializing...");
        telemetry.update();

        telemetry.setMsTransmissionInterval(50); // default is 250

        robot = new Robot(hardwareMap);
        //teamPropDetector = new TeamPropDetector(telemetry, hardwareMap);

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
                teamPropLocation = Location.LEFT;
            else if(gamepad1.dpad_up || gamepad2.dpad_up)
                teamPropLocation = Location.CENTER;
            else if(gamepad1.dpad_right || gamepad2.dpad_right)
                teamPropLocation = Location.RIGHT;
            //else if(gamepad1.a)
                //teamPropDetector.toggleOverlay();

            //teamPropDetector.setAlliance(alliance);
            //teamPropLocation = teamPropDetector.getLocation();

            telemetry.addLine("Initialized");
            telemetry.addLine(String.format("Alliance: %s (x = blue, b = red)", alliance));
            telemetry.addLine(String.format("Side: %s (left/right bumper)", side));
            //telemetry.addLine(String.format("Show overlay: %s (a)", teamPropDetector.isOverlayShown()));
            telemetry.addLine();
            telemetry.addData("Team prop location", teamPropLocation);
            //telemetry.addData("Color difference", teamPropDetector.getMinDifference());
            telemetry.addData("\ntelemetry transmission interval", telemetry.getMsTransmissionInterval());
            telemetry.update();
        }

        telemetry.setMsTransmissionInterval(250);

        // trajectories
        Pose2d startPose = new Pose2d();
        Trajectory spikeMarkTraj;
        Pose2d parkCoords = new Pose2d();
        int multiplier = side == Side.RIGHT ? 1 : -1;
        if(alliance == Alliance.BLUE && side == Side.LEFT) {
            startPose = new Pose2d(15, 62.5, -Math.PI / 2);

            parkCoords = new Pose2d(61, 57, Math.PI);

            if(teamPropLocation == Location.LEFT) {
                spikeMarkTraj = robot.drive.trajectoryBuilder(startPose)
                        .splineToLinearHeading(new Pose2d(21, 28, -Math.PI / 2), -Math.PI / 2)
                        .build();
            } else if(teamPropLocation == Location.CENTER) {
                spikeMarkTraj = robot.drive.trajectoryBuilder(startPose)
                        .splineToLinearHeading(new Pose2d(13, 21, Math.PI), Math.PI)
                        .build();
                parkCoords = parkCoords.plus(new Pose2d(0, 1, 0));
                telemetry.addData("new park coords", parkCoords);
                telemetry.update();
            } else
                spikeMarkTraj = robot.drive.trajectoryBuilder(startPose)
                        .splineToLinearHeading(new Pose2d(.5, 27, -Math.PI / 2), -Math.PI / 2)
                        .build();
        } else {
            spikeMarkTraj = robot.drive.trajectoryBuilder(startPose)
                    .splineToLinearHeading(parkCoords, 0)
                    .build();
        }
        double intakeWaitTime = 1;

        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(startPose)
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
        Trajectory park = robot.drive.trajectoryBuilder(traj.end(), true)
                .splineToLinearHeading(parkCoords, 0) // park in backstage
                .build();

        robot.drive.followTrajectorySequence(traj);
        robot.drive.followTrajectory(park);
    }
}