package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Motor;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends LinearOpMode {
    private static boolean singleDriverMode = false;
    public static double SLOW_SPEED = .3;
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        Robot robot = new Robot(hardwareMap);
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int direction = 1;
        double speed = 1;

        ElapsedTime armTimer = new ElapsedTime(), hangTimer = new ElapsedTime();

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

            // reverse direction
            if((!singleDriverMode && gamepad1.left_trigger > .1) || (gamepad1.back && gamepad1.left_trigger > .1))
                direction = -1;
            else if((!singleDriverMode && gamepad1.right_trigger > .1) || (gamepad1.back && gamepad1.right_trigger > .1))
                direction = 1;

            if((!singleDriverMode && gamepad1.right_bumper) || gamepad1.left_stick_button || gamepad1.right_stick_button)
                speed = SLOW_SPEED;
            else
                speed = 1;

            if((gamepad1.back && gamepad1.a) || (gamepad2.back && gamepad2.a))
                singleDriverMode = true;
            else if((gamepad1.back && gamepad1.b) || (gamepad2.back && gamepad2.b))
                singleDriverMode = false;

            if(singleDriverMode)
                gamepad = gamepad1;
            else
                gamepad = gamepad2;

            // use the triggers to control the intake - right trigger intakes, left trigger outtakes
            if(gamepad.right_trigger > 0 && !gamepad.back)
                robot.intake.in(gamepad.right_trigger);
            else if(gamepad.left_trigger > 0 && !gamepad.back)
                robot.intake.out(gamepad.left_trigger);
            else
                robot.intake.stop();

            // bumpers raise/lower the intake
            if(gamepad.right_bumper && !gamepad.back)
                robot.intake.lower();
            else if(gamepad.left_bumper && !gamepad.back)
                robot.intake.raise();

            // a/b and x/y control the 2 claws
            if(gamepad.a && !gamepad.start && !gamepad.back)
                robot.claw1.down();
            else if(gamepad.b && !gamepad.start && !gamepad.back)
                robot.claw1.up();
            else if(gamepad.x && !gamepad.start && !gamepad.back)
                robot.claw2.down();
            else if(gamepad.y && !gamepad.start && !gamepad.back)
                robot.claw2.up();

            // left/right on d-pad rotate
            if(gamepad.dpad_right && gamepad.back)
                robot.outtake.rotator.rotateIncrementally();
            else if(gamepad.dpad_left && gamepad.back)
                robot.outtake.rotator.retractIncrementally();
            else if(gamepad.dpad_right)
                robot.outtake.rotator.rotateFully();
            else if(gamepad.dpad_left)
                robot.outtake.rotator.retractFully();

            // raise outtake
            if((gamepad.dpad_up && !gamepad.back) || (gamepad.dpad_down && !gamepad.back))
                robot.outtake.runUsingEncoder();
            else if(gamepad2.left_stick_y != 0 || gamepad.dpad_up || gamepad.dpad_down)
                robot.outtake.runWithoutEncoder();

            if (robot.outtake.isUsingEncoder()) {
                if (gamepad.start && gamepad.dpad_up)
                    robot.outtake.setUpPosition();
                else if (gamepad.start && gamepad.dpad_down)
                    robot.outtake.setDownPosition();
                else if (gamepad.dpad_up) {
                    robot.outtake.stopHolding();
                    robot.outtake.goToUpPosition();
                    armTimer.reset();
                } else if (gamepad.dpad_down) {
                    robot.outtake.stopHolding();
                    robot.outtake.goToDownPosition();
                    armTimer.reset();
                } else if (robot.outtake.isIdle() || armTimer.seconds() > 2) {
                    if(robot.outtake.getStatus().equals(Motor.Status.DOWN)) {
                        robot.outtake.stopHolding();
                        robot.outtake.stop();
                    } else
                        robot.outtake.hold();
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
                } else if(robot.outtake.getStatus().equals(Motor.Status.DOWN)) {
                    robot.outtake.stopHolding();
                    robot.outtake.stop();
                } else
                    robot.outtake.hold();
            }

            // hang
            if(gamepad.start && gamepad.right_stick_y < 0)
                robot.hangSubsystem.setUpPosition();
            else if(gamepad.start && gamepad.right_stick_y > 0)
                robot.hangSubsystem.setDownPosition();
            else if(gamepad2.right_stick_y != 0 && gamepad2.back)
                robot.hangSubsystem.runUsingEncoder();
            else if((gamepad2.right_stick_y != 0 && !gamepad2.back && !gamepad2.start) || (gamepad1.back && gamepad1.right_stick_y != 0))
                robot.hangSubsystem.runWithoutEncoder();

            if(robot.hangSubsystem.isUsingEncoder()) {
                if(gamepad2.right_stick_y < 0) {
                    robot.hangSubsystem.goToUpPosition();
                    hangTimer.reset();
                } else if(gamepad2.right_stick_y > 0) {
                    robot.hangSubsystem.goToDownPosition();
                    hangTimer.reset();
                } else if (robot.hangSubsystem.isIdle() || hangTimer.seconds() > 6)
                    robot.hangSubsystem.stop();
            } else if(!singleDriverMode || gamepad1.back)
                robot.hangSubsystem.setPower(-gamepad.right_stick_y);

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
            else if(!singleDriverMode && gamepad1.dpad_right)
                robot.hangSubsystem.rotateUp();
            else if(!singleDriverMode && gamepad1.dpad_left)
                robot.hangSubsystem.rotateDown();

            // gamepad1 a/b control launcher
            if((gamepad1.a && !singleDriverMode && !gamepad1.start && !gamepad1.back) || (gamepad.right_bumper && gamepad.back))
                robot.launcher.launch();
            else if((gamepad1.b && !singleDriverMode && !gamepad1.start && !gamepad1.back) || (gamepad.left_bumper && gamepad.back))
                robot.launcher.reset();

            // auto claw
            if(gamepad.back && gamepad.y)
                robot.autoClaw.out();
            else if(gamepad.back && gamepad.x)
                robot.autoClaw.in();

            telemetry.addData("Single driver mode (back + a/b)", singleDriverMode);
            telemetry.addData("Direction (RT/LT)", direction == 1 ? "forward (intake)" : "reverse (outtake)");
            telemetry.addData("Wheel speed (RB)", speed);
//            telemetry.addData("\nGamepad1 left_stick_y", gamepad1.left_stick_y);
//            telemetry.addData("Average wheel power", robot.drive.getAverageDriveMotorPower());
            telemetry.addData("\nINTAKE (triggers)", robot.intake.getPowerAsString());
            telemetry.addData("   raise/lower (bumpers)", "%s (%.2f)", robot.intake.getStatus().toString().toLowerCase(), robot.intake.lowerer.getPosition());
            telemetry.addData("CLAW 1 (a/b)", "%s (%.2f)", robot.claw1.getStatus(), robot.claw1.getPosition());
            telemetry.addData("CLAW 2 (x/y)", "%s (%.2f)", robot.claw2.getStatus(), robot.claw2.getPosition());
            telemetry.addData("OUTTAKE (d-pad)", "%s %s (%d) (%s)", robot.outtake.getStatus().toString().toLowerCase(), robot.outtake.isHolding() ? "(holding at " + robot.outtake.getHoldPosition() + ")" : "", robot.outtake.getPosition(), robot.outtake.getPowerAsString());
            telemetry.addData("- Rotator", "%s (%.2f)", robot.outtake.rotator.getStatus(), robot.outtake.rotator.getPosition());
            telemetry.addData("HANG (right stick)", "%s (%s, %d) %s", robot.hangSubsystem.getStatus().toString().toLowerCase(), robot.hangSubsystem.getPowerAsString(), robot.hangSubsystem.getPosition(), robot.hangSubsystem.isUsingEncoder() ? "(encoder)" : "");
            telemetry.addData("- Rotator", "%s (%.2f)", robot.hangSubsystem.getRotatorStatus(), robot.hangSubsystem.rotator.getPosition());
            telemetry.addData("LAUNCHER (gmpd 1 a/b)", "%s (%.2f)", robot.launcher.getStatus(), robot.launcher.getPosition());
            telemetry.addData("AUTO CLAW (back + x/y)", "%s (%.2f)", robot.autoClaw.getStatus(), robot.autoClaw.getPosition());
            telemetry.update();
        }
    }
}
