package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends LinearOpMode {
    private static boolean singleDriverMode = false;
    public static double SLOW_SPEED = .5;
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

            if(gamepad1.back && gamepad1.a)
                singleDriverMode = true;
            else if((gamepad1.back && gamepad1.b) || (gamepad2.back && gamepad2.b))
                singleDriverMode = false;
        }

        // as soon as it starts, lower intake
        //robot.intake.lower();

        while (opModeIsActive() && !(gamepad1.start && gamepad1.back) && !(gamepad2.start && gamepad2.back)) {
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            direction * speed * gamepad1.left_stick_y,
                            direction * speed * gamepad1.left_stick_x,
                            speed * -gamepad1.right_stick_x
                    )
            );
            robot.drive.update();

            // reverse direction
            if((!singleDriverMode && gamepad1.left_trigger > .1) || (gamepad1.back && gamepad1.left_trigger > .1))
                direction = -1;
            else if((!singleDriverMode && gamepad1.right_trigger > .1) || (gamepad1.back && gamepad1.right_trigger > .1))
                direction = 1;

            if((!singleDriverMode && gamepad1.right_bumper) || gamepad1.start)
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
            if(gamepad.right_bumper && gamepad.back)
                robot.intake.lower(.1);
            else if(gamepad.left_bumper && gamepad.back)
                robot.intake.raise(.1);
            else if(gamepad.right_bumper)
                robot.intake.lower();
            else if(gamepad.left_bumper)
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
                robot.rotator.rotateIncrementally();
            else if(gamepad.dpad_left && gamepad.back)
                robot.rotator.retractIncrementally();
            else if(gamepad.dpad_right)
                robot.rotator.rotateFully();
            else if(gamepad.dpad_left)
                robot.rotator.retractFully();

            // raise outtake
            if((gamepad.dpad_up && !gamepad.back) || (gamepad.dpad_down && !gamepad.back))
                robot.outtakeRaiser.runUsingEncoder();
            else if(gamepad2.left_stick_y != 0 || gamepad.dpad_up || gamepad.dpad_down)
                robot.outtakeRaiser.runWithoutEncoder();

            if (robot.outtakeRaiser.isUsingEncoder()) {
                if (gamepad.start && gamepad.dpad_up)
                    robot.outtakeRaiser.setUpPosition();
                else if (gamepad.start && gamepad.dpad_down)
                    robot.outtakeRaiser.setDownPosition();
                else if (gamepad.dpad_up) {
                    robot.outtakeRaiser.stopHolding();
                    robot.outtakeRaiser.goToUpPosition();
                    armTimer.reset();
                } else if (gamepad.dpad_down) {
                    robot.outtakeRaiser.stopHolding();
                    robot.outtakeRaiser.goToDownPosition();
                    armTimer.reset();
                } else if (robot.outtakeRaiser.isIdle() || armTimer.seconds() > 2)
                    robot.outtakeRaiser.hold();
            } else {
                if(gamepad2.left_stick_y != 0) {
                    robot.outtakeRaiser.stopHolding();
                    robot.outtakeRaiser.setPower(-gamepad2.left_stick_y);
                } else if(gamepad.dpad_up) {
                    robot.outtakeRaiser.stopHolding();
                    robot.outtakeRaiser.accelerateUp();
                } else if(gamepad.dpad_down) {
                    robot.outtakeRaiser.stopHolding();
                    robot.outtakeRaiser.accelerateDown();
                } else if(robot.outtakeRaiser.getPower() != 0)
                    robot.outtakeRaiser.decelerate();
                else
                    robot.outtakeRaiser.hold();
            }

            // hang
            if(gamepad2.right_stick_y != 0 && gamepad2.back)
                robot.hangMotor.runUsingEncoder();
            else if((gamepad2.right_stick_y != 0 && !gamepad2.back && !gamepad2.start) || (gamepad1.back && gamepad1.right_stick_y != 0))
                robot.hangMotor.runWithoutEncoder();

            if(robot.hangMotor.isUsingEncoder()) {
                if(gamepad2.start && gamepad2.right_stick_y < 0)
                    robot.hangMotor.setUpPosition();
                else if(gamepad2.start && gamepad2.right_stick_y > 0)
                    robot.hangMotor.setDownPosition();
                else if(gamepad2.right_stick_y < 0) {
                    robot.hangMotor.goToUpPosition();
                    hangTimer.reset();
                } else if(gamepad2.right_stick_y > 0) {
                    robot.hangMotor.goToDownPosition();
                    hangTimer.reset();
                } else if (robot.hangMotor.isIdle() || hangTimer.seconds() > 6)
                    robot.hangMotor.stop();
            } else {
                robot.hangMotor.setPower(-gamepad.right_stick_y);
            }

            // auto claw
            if(gamepad.back && gamepad.y)
                robot.autoClaw.outIncrementally();
            else if(gamepad.back && gamepad.x)
                robot.autoClaw.inIncrementally();

            // gamepad1 a/b control launcher
//            if(gamepad1.a && !singleDriverMode && !gamepad1.start && !gamepad1.back)
//                robot.launcher.launch();
//            else if(gamepad1.b && !singleDriverMode && !gamepad1.start && !gamepad1.back)
//                robot.launcher.reset();

            telemetry.addData("Single driver mode (back + a/b)", singleDriverMode);
            telemetry.addData("Direction (RT/LT)", direction == 1 ? "forward (intake)" : "reverse (outtake)");
            telemetry.addData("Wheel speed (RB)", speed);
//            telemetry.addData("\nGamepad1 left_stick_y", gamepad1.left_stick_y);
//            telemetry.addData("Average wheel power", robot.drive.getAverageDriveMotorPower());
            telemetry.addData("\nIntake (triggers)", robot.intake.getPowerAsString());
            telemetry.addData("   raise/lower (bumpers)", "%s (%.2f)", robot.intake.getStatus(), robot.intake.lowerer.getPosition());
            telemetry.addData("Claw 1 (a/b)", "%s (%.2f)", robot.claw1.getStatus(), robot.claw1.getPosition());
            telemetry.addData("Claw 2 (x/y)", "%s (%.2f)", robot.claw2.getStatus(), robot.claw2.getPosition());
            telemetry.addData("Rotator (d-pad L/R)", "%s (%.2f)", robot.rotator.getStatus(), robot.rotator.getPosition());
            telemetry.addData("Raise outtake (d-pad U/D or left stick)", "%d (%s) (pwr: %s)", robot.outtakeRaiser.getPosition(), robot.outtakeRaiser.getStatus(), robot.outtakeRaiser.getPowerAsString());
            telemetry.addData("Hang (right stick y)", "%s (%d) %s", robot.hangMotor.getPowerAsString(), robot.hangMotor.getPosition(), robot.hangMotor.isUsingEncoder() ? "(using encoder)" : "");
            telemetry.addData("Auto claw (back + x/y)", "%s (%.2f)", robot.autoClaw.getStatus(), robot.autoClaw.getPosition());
//            telemetry.addData("Launcher psn", "%.2f", robot.launcher.getPosition());
            telemetry.update();
        }
    }
}
