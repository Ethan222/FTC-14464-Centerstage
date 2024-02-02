package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Motor;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeRotator;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends LinearOpMode {
    private static boolean singleDriverMode = false;
    public static double SLOW_SPEED = .3;
    private ScheduledExecutorService executorService;
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        Robot robot = new Robot(hardwareMap);
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.setPoseEstimate(new Pose2d());
        robot.launcher.rotator.setPosition(robot.launcher.rotator.getRightPosition());

        executorService = Executors.newSingleThreadScheduledExecutor();

        ElapsedTime wheelTimer = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();

        int direction = 1;
        double speed = 1, heading;

        Gamepad gamepad;

        while(opModeInInit() && !(gamepad1.start && gamepad1.back) && !(gamepad2.start && gamepad2.back)) {
            telemetry.addLine("Initialized");
            telemetry.addData("Single driver mode (back + a/b)", singleDriverMode);
            telemetry.update();

            if((gamepad1.back && gamepad1.a) || (gamepad2.back && gamepad2.a))
                singleDriverMode = true;
            else if((gamepad1.back && gamepad1.b) || (gamepad2.back && gamepad2.b))
                singleDriverMode = false;
        }

        // as soon as it starts, lower intake
        if(!singleDriverMode)
            robot.intake.lower();

        while (opModeIsActive() && !(gamepad1.start && gamepad1.back) && !(gamepad2.start && gamepad2.back)) {
            if(!singleDriverMode || !gamepad1.back) {
                robot.drive.setWeightedDrivePower(
                        new Pose2d(
                                direction * speed * gamepad1.left_stick_y,
                                direction * speed * gamepad1.left_stick_x,
                                speed * -gamepad1.right_stick_x
                        )
                );
            }
            robot.drive.update();

            heading = robot.drive.getPoseEstimate().getHeading();
            if(heading > Math.PI)
                heading -= 2 * Math.PI;
            telemetry.addData("Heading", "%.1f deg", heading * 180 / Math.PI);
            Telemetry.Item item = telemetry.addData("turning", "");

            if(gamepad1.dpad_down && !singleDriverMode && wheelTimer.seconds() > 1 && Math.abs(heading) > .1) {
                double newHeading = -heading;
                item.setValue("%.0f deg", newHeading * 180 / Math.PI);
                robot.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.drive.turnAsync(heading);
                wheelTimer.reset();
            }
            telemetry.addLine();

            // reverse direction
            if((!singleDriverMode && gamepad1.left_bumper) || (gamepad1.start && gamepad1.left_bumper))
                direction = -1;
            else if((!singleDriverMode && gamepad1.right_bumper) || (gamepad1.start && gamepad1.right_bumper))
                direction = 1;

            // slow mode
            if((!singleDriverMode && gamepad1.right_trigger > .1) || gamepad1.left_stick_button || gamepad1.right_stick_button || gamepad1.start)
                speed = SLOW_SPEED;
            else
                speed = 1;

            // single drive mode
            if((gamepad1.back && gamepad1.a) || (gamepad2.back && gamepad2.a))
                singleDriverMode = true;
            else if((gamepad1.back && gamepad1.b) || (gamepad2.back && gamepad2.b))
                singleDriverMode = false;

            if(singleDriverMode)
                gamepad = gamepad1;
            else
                gamepad = gamepad2;

            // use the triggers to control the intake - right trigger intakes, left trigger outtakes
            // can only intake if arm is down and outtake is retracted
            if(gamepad.start && gamepad.right_trigger > 0) {
                robot.outtake.rotator.retractFully();
                robot.outtake.setDownPosition();
                robot.claw1.up();
                robot.claw2.up();
            }
            if(gamepad.right_trigger > 0 && !gamepad.back &&
                    robot.outtake.getState() == Motor.State.DOWN && robot.outtake.rotator.getState().equals(OuttakeRotator.RETRACTED))
                robot.intake.in(gamepad.right_trigger);
            else if(gamepad.left_trigger > 0 && !gamepad.back)
                robot.intake.out(gamepad.left_trigger);
            else
                robot.intake.stop();

            // back + triggers raise/lower the intake
            if(gamepad.right_trigger > .8 && gamepad.back)
                robot.intake.lower();
            else if(gamepad.left_trigger > .8 && gamepad.back)
                robot.intake.raise();
            else if(gamepad.right_trigger > 0 && gamepad.back)
                robot.intake.lower(.01);
            else if(gamepad.left_trigger > 0 && gamepad.back)
                robot.intake.raise(.01);

            // a/b and x/y control the 2 claws
            if(gamepad.a && !gamepad.start && !gamepad.back)
                robot.claw1.down();
            else if(gamepad.b && !gamepad.start && !gamepad.back)
                robot.claw1.up();
            if(gamepad.x && !gamepad.start && !gamepad.back)
                robot.claw2.down();
            else if(gamepad.y && !gamepad.start && !gamepad.back)
                robot.claw2.up();

            // raise outtake
            if((gamepad.dpad_up && !gamepad.back) || (gamepad.dpad_down && !gamepad.back))
                robot.outtake.runUsingEncoder();
            else if(gamepad2.left_stick_y != 0 || gamepad.dpad_up || gamepad.dpad_down)
                robot.outtake.runWithoutEncoder();

            if (robot.outtake.isUsingEncoder()) {
                if (gamepad.dpad_up && gamepad.start)
                    robot.outtake.setUpPosition();
                else if (gamepad.dpad_down && gamepad.start)
                    robot.outtake.setDownPosition();
                else if (gamepad.dpad_up) {
                    // when arm goes up, also rotate outtake & close claws
                    robot.claw1.down();
                    robot.claw2.down();
                    if(!robot.outtake.rotator.getState().equals(OuttakeRotator.RETRACTED))
                        robot.outtake.rotator.rotateFully();
                    robot.outtake.stopHolding();
                    if(robot.outtake.getPosition() < Outtake.POSITION_1)
                        robot.outtake.goToPosition(Outtake.POSITION_1);
                    else
                        robot.outtake.goToPosition(Outtake.POSITION_2);
                } else if (gamepad.dpad_down) {
                    robot.outtake.stopHolding();
                    robot.outtake.goToDownPosition();
                    executorService.schedule(robot.outtake::stop, 2, TimeUnit.SECONDS);
                    // when arm goes down, also retract outtake
                    robot.outtake.rotator.retractFully();
                } else if (robot.outtake.isIdle()) {
                    robot.outtake.stopHolding();
                    robot.outtake.stop();
                }
            } else {
                if(gamepad2.left_stick_y != 0) {
                    robot.outtake.stopHolding();
                    robot.outtake.setPower(-gamepad2.left_stick_y);
                } else if(gamepad.dpad_up) {
                    robot.outtake.stopHolding();
                    robot.outtake.accelerateUp();
                } else if(gamepad.dpad_down) {
                    robot.outtake.stopHolding();
                    robot.outtake.accelerateDown();
                } else //if(robot.outtake.getStatus().equals(Motor.State.DOWN)) {
//                    robot.outtake.stopHolding();
                    robot.outtake.decelerate();
//                } else
//                    robot.outtake.hold();
            }
            double armSlowSpeed = .3;
            if(robot.outtake.getPosition() - robot.outtake.getDownPosition() < 400 && robot.outtake.getPower() < -armSlowSpeed)
                robot.outtake.setPower(-armSlowSpeed);

            // rotate outtake
            if(gamepad.dpad_right && gamepad.start)
                robot.outtake.rotator.EXTENDED_PSN = robot.outtake.rotator.getPosition();
            else if(gamepad.dpad_left && gamepad.start)
                robot.outtake.rotator.RETRACTED_PSN = robot.outtake.rotator.getPosition();
            else if(gamepad.dpad_right && gamepad.back)
                robot.outtake.rotator.rotateIncrementally();
            else if(gamepad.dpad_left && gamepad.back)
                robot.outtake.rotator.retractIncrementally();
            else if(gamepad.dpad_right) {
                // make sure both claws are down first
                robot.claw1.down();
                robot.claw2.down();
                robot.outtake.rotator.rotateFully();
            } else if(gamepad.dpad_left)
                robot.outtake.rotator.retractFully();

            // hang
//            if(gamepad.start && gamepad.right_stick_y < 0)
//                robot.hangSubsystem.setUpPosition();
//            else if(gamepad.start && gamepad.right_stick_y > 0)
//                robot.hangSubsystem.setDownPosition();
//            else if(gamepad2.right_stick_y != 0 && gamepad2.back)
//                robot.hangSubsystem.runUsingEncoder();
//            else if((gamepad2.right_stick_y != 0 && !gamepad2.back && !gamepad2.start) || (gamepad1.back && gamepad1.right_stick_y != 0))
//                robot.hangSubsystem.runWithoutEncoder();

//            if(robot.hangSubsystem.isUsingEncoder()) {
//                if(gamepad2.right_stick_y < 0) {
//                    robot.hangSubsystem.goToUpPosition();
//                    hangTimer.reset();
//                } else if(gamepad2.right_stick_y > 0) {
//                    robot.hangSubsystem.goToDownPosition();
//                    hangTimer.reset();
//                } else if (robot.hangSubsystem.isIdle() || hangTimer.seconds() > 6)
//                    robot.hangSubsystem.stop();
            if((!singleDriverMode || gamepad1.back) && gamepad.right_stick_x < .3) {
//                robot.hangSubsystem.setPowers(-gamepad.right_stick_y);
                if(Math.abs(gamepad.right_stick_y) == 1)
                    robot.hangSubsystem.setPowers(.8 * -gamepad.right_stick_y);
                else if(Math.abs(gamepad.right_stick_y) > .4)
                    robot.hangSubsystem.setPowers(.5 * gamepad.right_stick_y < 0 ? 1 : -1);
                else if(Math.abs(gamepad.right_stick_y) > 0)
                    robot.hangSubsystem.setPowers(.2 * gamepad.right_stick_y < 0 ? 1 : -1);
                else
                    robot.hangSubsystem.setPowers(0);
                if(gamepad.right_stick_y < -.9 && robot.hangSubsystem.getRotatorState().equals(HangSubsystem.DOWN))
                    robot.hangSubsystem.rotateUp();
            }

            // rotate hang
            double threshold = .8;
            if(gamepad2.back && gamepad2.right_stick_x > threshold)
                robot.hangSubsystem.rotateUp(.01);
            else if(gamepad2.back && gamepad2.right_stick_x < -threshold)
                robot.hangSubsystem.rotateDown(.01);
            else if(gamepad2.right_stick_x > threshold || (singleDriverMode && gamepad1.back && gamepad1.right_stick_x > threshold))
                robot.hangSubsystem.rotateUp();
            else if(gamepad2.right_stick_x < -threshold || (singleDriverMode && gamepad1.back && gamepad1.right_stick_x < -threshold))
                robot.hangSubsystem.rotateDown();

            // launcher
            if((gamepad1.a && !singleDriverMode && !gamepad1.start && !gamepad1.back) || (gamepad.right_bumper && !gamepad.start && !gamepad.back))
                robot.launcher.launch();
            else if((gamepad1.b && !singleDriverMode && !gamepad1.start && !gamepad1.back) || (gamepad.left_bumper && !gamepad.start && !gamepad.back))
                robot.launcher.reset();
            if((gamepad.right_bumper && gamepad.back) || (!singleDriverMode && gamepad1.y))
                robot.launcher.rotateUp();
            else if((gamepad.left_bumper && gamepad.back) || (!singleDriverMode && gamepad1.x))
                robot.launcher.rotateDown();

            // auto claw
            if(gamepad.start && gamepad.y)
                robot.autoClaw.outAndIn();
            else if(gamepad.start && gamepad.x)
                robot.autoClaw.in();
            else if(gamepad.back && gamepad.y)
                robot.autoClaw.outIncrementally();
            else if(gamepad.back && gamepad.x)
                robot.autoClaw.inIncrementally();

            telemetry.addData("Single driver mode", singleDriverMode);
            telemetry.addData("Direction (RB/LB)", direction == 1 ? "forward (INTAKE)" : "reverse (OUTTAKE)");
            telemetry.addData("Wheel speed (RT)", speed);
//            telemetry.addData("Average wheel power", robot.drive.getAverageDriveMotorPower());
            telemetry.addData("\nintake", "%s (%.2f) [%s]", robot.intake.getState().toString(), robot.intake.lowerer.getPosition(), robot.intake.getPowerAsString());
            telemetry.addData("\nclaw 1", "%s (%.2f)", robot.claw1.getState(), robot.claw1.getPosition());
            telemetry.addData("claw 2", "%s (%.2f)", robot.claw2.getState(), robot.claw2.getPosition());
            telemetry.addData("outtake", "%s %s (%d) [%s]", robot.outtake.getState().toString(), robot.outtake.isHolding() ? "(holding at " + robot.outtake.getHoldPosition() + ")" : "", robot.outtake.getPosition(), robot.outtake.getPowerAsString());
            telemetry.addData("- rotator", "%s (%.3f)", robot.outtake.rotator.getState(), robot.outtake.rotator.getPosition());
            telemetry.addData("\nhang", "[%s] (%s)", robot.hangSubsystem.getPowersAsString(), robot.hangSubsystem.getPositionsAsStrings());
            telemetry.addData("- rotator", "%s (%.2f)", robot.hangSubsystem.getRotatorState(), robot.hangSubsystem.rotator.getPosition());
            telemetry.addData("launcher", "%s [%.2f] (%.2f)", robot.launcher.getState(), robot.launcher.launcher.getPercent(), robot.launcher.launcher.getPosition());
            telemetry.addData("- rotator", "%s [%.2f] (%.2f)", robot.launcher.getRotationState(), robot.launcher.getRotationPercent(), robot.launcher.getRotation());
            telemetry.addData("\nauto claw", "%s (%.2f)", robot.autoClaw.getState(), robot.autoClaw.getPosition());
            telemetry.addData("\nLoop time", "%.0f ms", loopTimer.milliseconds());
//            telemetry.addLine().addData("Gamepad1 at rest", gamepad1.atRest()).addData("gamepad2 at rest", gamepad2.atRest());
            loopTimer.reset();
            telemetry.update();
        }
    }
}
