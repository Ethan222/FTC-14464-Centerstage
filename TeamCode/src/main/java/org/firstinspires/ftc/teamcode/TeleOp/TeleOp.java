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

        Button x = new Button(), y = new Button();

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
                            -Math.pow(gamepad1.left_stick_y, 1),
                            -Math.pow(gamepad1.left_stick_x, 1),
                            -Math.pow(gamepad1.right_stick_x, 1)
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

            // bumpers raise/lower the intake
            if(gamepad.right_bumper)
                robot.intake.lower();
            else if(gamepad.left_bumper)
                robot.intake.raise();

            // use the triggers to control the intake - right trigger intakes, left trigger outtakes
            if(gamepad.right_trigger > 0)
                robot.intake.in(gamepad.right_trigger);
            else if(gamepad.left_trigger > 0)
                robot.intake.out(gamepad.left_trigger);
            else
                robot.intake.stop();

            // left stick y or d-pad up/down controls the arm lifter
            if(gamepad2.left_stick_y != 0)
                robot.armRaiser.setPower(-gamepad2.left_stick_y);
            else if(gamepad.dpad_up)
                robot.armRaiser.up();
            else if(gamepad.dpad_down)
                robot.armRaiser.down();
            else
                robot.armRaiser.stop();

            // a/b grip/ungrip fully
//            if(gamepad2.a && !gamepad2.start && !gamepad2.back)
//                robot.gripper.gripFully();
//            else if(gamepad2.b && !gamepad2.start && !gamepad2.back)
//                robot.gripper.ungripFully();
//            else if(singleDriverMode && gamepad1.a && !gamepad1.start && !gamepad1.back)
//                robot.gripper.gripFully();
//            else if(singleDriverMode && gamepad1.b && !gamepad1.start && !gamepad1.back)
//                robot.gripper.ungripFully();

            // x/y rotate
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

            // gamepad1 a/b control launcher
//            if(gamepad1.a && !singleDriverMode && !gamepad1.start && !gamepad1.back)
//                robot.launcher.launch();
//            else if(gamepad1.b && !singleDriverMode && !gamepad1.start && !gamepad1.back)
//                robot.launcher.reset();

            telemetry.addData("Single driver mode", singleDriverMode);
            telemetry.addLine("\nintake: triggers (raise/lower intake: bumpers)");
            telemetry.addLine("arm: left joystick or d-pad up/down");
//            telemetry.addLine("gripper: a/b");
//            telemetry.addLine("rotator: x/y");
//            telemetry.addLine("launcher: gamepad1 a/b");
            telemetry.addData("\nGamepad1 left_stick_y", gamepad1.left_stick_y);
            telemetry.addData("Average wheel power", robot.drive.getAverageDriveMotorPower());
            telemetry.addData("\nIntake psn", "%s (%.1f)", robot.intake.getStatus(), robot.intake.lowerer.getPosition());
            telemetry.addData("Intake pwr", robot.intake.getPower());
//            telemetry.addData("Arm flipper pwr", robot.armFlipper.getPower());
            telemetry.addData("Arm raiser pwr", robot.armRaiser.getPower());
//            telemetry.addData("Rotator pwr", "%.2f", robot.rotator.getPower());
//            telemetry.addData("Gripper psn", "%s (%.3f)", robot.gripper.getStatus(), robot.gripper.getPosition());
//            telemetry.addData("Launcher psn", "%.2f", robot.launcher.getPosition());
            telemetry.update();
        }
    }
}
