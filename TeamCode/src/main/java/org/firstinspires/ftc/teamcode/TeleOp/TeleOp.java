package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends LinearOpMode {
    private static boolean singleDriverMode = false;
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        Robot robot = new Robot(hardwareMap);
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Button x = new Button(), y = new Button();

        int reverseDirection = -1;

        Gamepad gamepad;
        while(opModeInInit()) {
            telemetry.addLine("Initialized");
            telemetry.addData("Single driver mode", singleDriverMode);
            telemetry.update();

            if(gamepad1.back && gamepad1.a)
                singleDriverMode = true;
            else if((gamepad1.back && gamepad1.b) || (gamepad2.back && gamepad2.b))
                singleDriverMode = false;
            else if((gamepad1.start && gamepad1.x) || (gamepad2.start && gamepad2.x))
                requestOpModeStop();
        }

        // as soon as it starts, lower intake
        //robot.intake.lower();

        while (opModeIsActive() && !(gamepad1.start && gamepad1.x) && !(gamepad2.start && gamepad2.x)) {
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            reverseDirection * Math.pow(gamepad1.left_stick_y, 1),
                            reverseDirection * Math.pow(gamepad1.left_stick_x, 1),
                            reverseDirection * Math.pow(gamepad1.right_stick_x, 1)
                    )
            );
            robot.drive.update();

            if((gamepad1.back && gamepad1.a) || (gamepad2.back && gamepad2.a))
                singleDriverMode = true;
            else if((gamepad1.back && gamepad1.b) || (gamepad2.back && gamepad2.b))
                singleDriverMode = false;

            if(singleDriverMode)
                gamepad = gamepad1;
            else
                gamepad = gamepad2;

            if(gamepad1.start && gamepad1.y)
                reverseDirection *= -1;

            // use the triggers to control the intake - right trigger intakes, left trigger outtakes
            if(gamepad.right_trigger > 0)
                robot.intake.in(gamepad.right_trigger);
            else if(gamepad.left_trigger > 0)
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

            // a/b and x/y control the 2 grippers
            if(gamepad.a && !gamepad.start && !gamepad.back)
                robot.gripper1.downFully();
            else if(gamepad.b && !gamepad.start && !gamepad.back)
                robot.gripper1.upFully();
            else if(gamepad.x && !gamepad.start && !gamepad.back)
                robot.gripper2.downFully();
            else if(gamepad.y && !gamepad.back)
                robot.gripper2.upFully();

            // left/right on d-pad rotate
            if(gamepad.dpad_right && gamepad.back)
                robot.rotator.rotateIncrementally();
            else if(gamepad.dpad_left && gamepad.back)
                robot.rotator.retractIncrementally();
            else if(gamepad.dpad_right)
                robot.rotator.rotateFully();
            else if(gamepad.dpad_left)
                robot.rotator.retractFully();

            // short press moves at half speed, after half a second it starts moving at full speed
//            if(gamepad2.y || (singleDriverMode && gamepad1.y)) {
//                y.down();
//                if(y.getTimeDown() < 500) // short press
//                    robot.rotator.rotate(.5); // just move a little bit (small adjustments)
//                else // long press
//                    robot.rotator.rotate(1); // move as fast as possible
//            } else if(gamepad2.x || (singleDriverMode && gamepad1.x)) {
//                x.down();
//                if(x.getTimeDown() < 500)
//                    robot.rotator.unrotate(.5);
//                else
//                    robot.rotator.unrotate(1);
//            } else {
//                x.up();
//                y.up();
//                robot.rotator.stop();
//            }

            // left stick y or d-pad up/down raises the outtake
            if(gamepad2.left_stick_y != 0)
                robot.outtakeRaiser.setPower(-gamepad2.left_stick_y);
            else if(gamepad.dpad_up)
                robot.outtakeRaiser.up();
            else if(gamepad.dpad_down)
                robot.outtakeRaiser.down();
            else
                robot.outtakeRaiser.stop();

            // hang
            if(!singleDriverMode || (singleDriverMode && gamepad1.back))
                robot.hangMotor.setPower(-gamepad.right_stick_y);

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
            telemetry.addData("Reverse direction (start + y)", reverseDirection == 1);
            telemetry.addData("\nGamepad1 left_stick_y", gamepad1.left_stick_y);
            telemetry.addData("Average wheel power", robot.drive.getAverageDriveMotorPower());
            telemetry.addData("\nIntake (triggers)", robot.intake.getPower());
            telemetry.addData(" raise/lower (bumpers)", "%s (%.2f)", robot.intake.getStatus(), robot.intake.lowerer.getPosition());
            telemetry.addData("Gripper1 (a/b)", "%s (%.3f)", robot.gripper1.getStatus(), robot.gripper1.getPosition());
            telemetry.addData("Gripper2 (x/y)", "%s (%.3f)", robot.gripper2.getStatus(), robot.gripper2.getPosition());
            telemetry.addData("Rotator (d-pad L/R)", "%s (%.3f)", robot.rotator.getStatus(), robot.rotator.getPosition());
            telemetry.addData("Raise outtake (left stick or d-pad U/D)", robot.outtakeRaiser.getPower());
            telemetry.addData("Hang (right stick y)", robot.hangMotor.getPower());
            telemetry.addData("Auto claw (back + x/y)", "%s (%.3f)", robot.autoClaw.getStatus(), robot.autoClaw.getPosition());
//            telemetry.addData("Launcher psn", "%.2f", robot.launcher.getPosition());
            telemetry.update();
        }
    }
}
