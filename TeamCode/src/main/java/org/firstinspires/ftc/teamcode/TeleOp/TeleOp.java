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

        int reverseDirection = 1;
        double speed = 1;

        boolean usingEncoder = true;
        ElapsedTime armTimer = new ElapsedTime();

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
                            reverseDirection * speed * gamepad1.left_stick_y,
                            reverseDirection * speed * gamepad1.left_stick_x,
                            speed * -gamepad1.right_stick_x
                    )
            );
            robot.drive.update();

            if(gamepad1.start && gamepad1.x)
                reverseDirection = 1;
            else if(gamepad1.start && gamepad1.y)
                reverseDirection = -1;

            if((!singleDriverMode && gamepad1.right_bumper) || gamepad1.start)
                speed = SLOW_SPEED;
            else
                speed = 1;

            if(gamepad1.back && gamepad1.a)
                singleDriverMode = true;
            else if(gamepad2.back && gamepad2.b)
                singleDriverMode = false;

            if(singleDriverMode)
                gamepad = gamepad1;
            else
                gamepad = gamepad2;

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
            if(gamepad.a && !gamepad.start && !gamepad.back) {
                robot.gripper1.downFully();
                reverseDirection = -1;
            } else if(gamepad.b && !gamepad.start && !gamepad.back) {
                robot.gripper1.upFully();
                reverseDirection = 1;
            } else if(gamepad.x && !gamepad.start && !gamepad.back)
                robot.gripper2.downFully();
            else if(gamepad.y && !gamepad.start && !gamepad.back)
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

            // raise outtake
            if(gamepad.dpad_up || gamepad.dpad_down)
                usingEncoder = true;
            else if(gamepad2.left_stick_y != 0)
                usingEncoder = false;

            if(usingEncoder) {
                if(gamepad.dpad_up) {
                    robot.outtakeRaiser.goToPosition1(this);
                    armTimer.reset();
                } else if (gamepad.dpad_down) {
                    robot.outtakeRaiser.goDown(this);
                    armTimer.reset();
                } else if (!robot.outtakeRaiser.isBusy() || armTimer.seconds() > 2)
                    robot.outtakeRaiser.stop();
            } else {
                robot.outtakeRaiser.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.outtakeRaiser.setPower(-gamepad2.left_stick_y);
            }

            // hang
            if(!singleDriverMode || gamepad1.back)
                robot.hangMotor.setPower(-gamepad.right_stick_y);

            // auto claw
            if(gamepad.back && gamepad.y)
                robot.autoClaw.out();
            else if(gamepad.back && gamepad.x)
                robot.autoClaw.in();

            // gamepad1 a/b control launcher
//            if(gamepad1.a && !singleDriverMode && !gamepad1.start && !gamepad1.back)
//                robot.launcher.launch();
//            else if(gamepad1.b && !singleDriverMode && !gamepad1.start && !gamepad1.back)
//                robot.launcher.reset();

            telemetry.addData("Single driver mode (back + a/b)", singleDriverMode);
            telemetry.addData("Reverse direction (start/back + y)", reverseDirection == -1);
            telemetry.addData("Wheel speed (RB or start)", speed);
//            telemetry.addData("\nGamepad1 left_stick_y", gamepad1.left_stick_y);
//            telemetry.addData("Average wheel power", robot.drive.getAverageDriveMotorPower());
            telemetry.addData("\nIntake (triggers)", robot.intake.getPower());
            telemetry.addData(" raise/lower (bumpers)", "%s (%.2f)", robot.intake.getStatus(), robot.intake.lowerer.getPosition());
            telemetry.addData("Gripper1 (a/b)", "%s (%.2f)", robot.gripper1.getStatus(), robot.gripper1.getPosition());
            telemetry.addData("Gripper2 (x/y)", "%s (%.2f)", robot.gripper2.getStatus(), robot.gripper2.getPosition());
            telemetry.addData("Rotator (d-pad L/R)", "%s (%.2f)", robot.rotator.getStatus(), robot.rotator.getPosition());
            telemetry.addData("Raise outtake (left stick or d-pad U/D)", "%s (%d)", robot.outtakeRaiser.getPower(), robot.outtakeRaiser.getPosition());
            telemetry.addData("Hang (right stick y)", robot.hangMotor.getPower());
            telemetry.addData("Auto claw (back + x/y)", "%s (%.2f)", robot.autoClaw.getStatus(), robot.autoClaw.getPosition());
//            telemetry.addData("Launcher psn", "%.2f", robot.launcher.getPosition());
            telemetry.update();
        }
    }
}
