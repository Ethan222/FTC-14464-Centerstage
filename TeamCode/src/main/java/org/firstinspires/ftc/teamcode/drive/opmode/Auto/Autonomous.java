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
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

    private Location teamPropLocation = Location.LEFT;

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
        Pose2d spikeMarkPosition = new Pose2d();
        Vector2d parkCoords = new Vector2d();
        if(alliance == Alliance.BLUE && side == Side.LEFT) {
            startPose = new Pose2d(11, 62.5, -Math.PI / 2);

            if(teamPropLocation == Location.LEFT)
                spikeMarkPosition = new Pose2d(22.5, 42);
            else if(teamPropLocation == Location.CENTER)
                spikeMarkPosition = new Pose2d(12, 38, Math.PI);
            else
                spikeMarkPosition = new Pose2d(.5, 42);

            parkCoords = new Vector2d(60, 62);
        }
        double waitTime = 1.5;
        int multiplier = side == Side.LEFT ? 1 : -1;

        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    robot.gripper.gripFully(); // grip the preloaded pixels
                })
                .waitSeconds(waitTime)
                .lineToLinearHeading(spikeMarkPosition) // move to spike mark
                .turn(teamPropLocation == Location.CENTER ? -Math.PI / 2 : 0)
                // place pixel on spike mark
                .addTemporalMarker(() -> {
                    robot.gripper.ungripFully();
                    robot.intake.out();
                })
                .back(10)
                .addTemporalMarker(() -> {
                    robot.intake.stop();
                })
                //.lineTo(parkCoords) // park in backstage
                .build();

        robot.drive.followTrajectorySequence(traj);
    }
}