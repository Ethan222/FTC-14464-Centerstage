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

    private Location teamPropLocation;

    @Override
    public void runOpMode()
    {
        telemetry.addLine("Initializing...");
        telemetry.update();

        //telemetry.setMsTransmissionInterval(50);

        robot = new Robot(hardwareMap);
        teamPropDetector = new TeamPropDetector(telemetry, hardwareMap);

        // init loop - select alliance and side
        while(!opModeIsActive() && !isStopRequested()) {
            if(gamepad1.x || gamepad2.x)
                alliance = Alliance.BLUE;
            else if(gamepad1.b || gamepad2.b)
                alliance = Alliance.RED;
            else if(gamepad1.left_bumper)
                side = Side.LEFT;
            else if(gamepad1.right_bumper)
                side = Side.RIGHT;
            else if(gamepad1.a)
                teamPropDetector.toggleOverlay();

            teamPropDetector.setAlliance(alliance);
            teamPropLocation = teamPropDetector.getLocation();

            telemetry.addLine(String.format("Alliance: %s (x = blue, b = red)", alliance));
            telemetry.addLine(String.format("Side: %s (left/right bumper)", side));
            telemetry.addLine(String.format("Show overlay: %s (a)", teamPropDetector.isOverlayShown()));
            telemetry.addLine();
            telemetry.addData("Team prop location", teamPropLocation);
            telemetry.addData("Color difference", teamPropDetector.getMinDifference());
            telemetry.update();
        }

        int location = 0;

        // trajectories
        Pose2d startPose = new Pose2d(11, 62.5, -Math.PI / 2);
        Vector2d[] spikeMarkPoses = {
                new Vector2d(22.5, 42),
                new Vector2d(12, 38),
                new Vector2d(.5, 42)
        };
        double forwardDistance = 26;
        double waitTime = 1.5;
        double[] backdropYs = {41, 35, 28};

        Vector2d parkCoords = new Vector2d(60, 62);
        int multiplier = -1;

        robot.drive.setPoseEstimate(startPose);

        double armFlipStartTime = .5, armFlipTime = 3.5;
        Vector2d backdropCoords = new Vector2d(44, backdropYs[location]);
        TrajectorySequence traj = robot.drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    robot.gripper.gripFully(); // grip the preloaded pixels
                })
                .waitSeconds(.5)
                .lineTo(spikeMarkPoses[location]) // move to spike mark
                .turn(Math.PI)
                // place pixel on spike mark
                .addTemporalMarker(armFlipStartTime, () -> {
                    robot.armFlipper.flip();
                })
                .waitSeconds(3)
                .addTemporalMarker(armFlipStartTime + armFlipTime, () -> {
                    robot.armFlipper.stop();
                    robot.gripper.ungripFully();
                })
                //.turn(multiplier * Math.PI / 2)
                //.lineTo(backdropCoords)
                //.waitSeconds(waitTime)
                //.back(4)
                //.strafeRight(multiplier * 23)
                //.lineTo(parkCoords) // park in backstage
                .build();

        robot.drive.followTrajectorySequence(traj);
    }
}