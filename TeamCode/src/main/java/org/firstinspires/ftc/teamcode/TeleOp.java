package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        Robot robot = new Robot(hardwareMap);
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Button x = new Button(), y = new Button();

        boolean singleDriverMode = false;
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

            // bumpers raise/lower the intake
            if(gamepad2.right_bumper || (singleDriverMode && gamepad1.right_bumper))
                robot.intake.lower();
            else if(gamepad2.left_bumper || (singleDriverMode && gamepad1.left_bumper))
                robot.intake.raise();

            // use the triggers to control the intake - right trigger intakes, left trigger outtakes
            if(gamepad2.right_trigger > 0)
                robot.intake.in(gamepad2.right_trigger);
            else if(gamepad2.left_trigger > 0)
                robot.intake.out(gamepad2.left_trigger);
            else if(singleDriverMode && gamepad1.right_trigger > 0)
                robot.intake.in(gamepad1.right_trigger);
            else if(singleDriverMode && gamepad1.left_trigger > 0)
                robot.intake.out(gamepad1.left_trigger);
            else
                robot.intake.stop();

            // right stick y or d-pad up/down controls the arm lifter
            if(gamepad2.right_stick_y != 0)
                robot.armRaisers.setPower(-gamepad2.right_stick_y);
            else if(gamepad2.dpad_up || (singleDriverMode && gamepad1.dpad_up))
                robot.armRaisers.up();
            else if(gamepad2.dpad_down || (singleDriverMode && gamepad1.dpad_down))
                robot.armRaisers.down();
            else
                robot.armRaisers.stop();

            // left stick y or d pad left/right controls arm flipper
            if(gamepad2.left_stick_y != 0)
                robot.armFlipper.flip(-gamepad2.left_stick_y);
            else if(gamepad2.dpad_right || (singleDriverMode && gamepad1.dpad_right))
                robot.armFlipper.flip();
            else if(gamepad2.dpad_left || (singleDriverMode && gamepad1.dpad_left))
                robot.armFlipper.unflip();
            else
                robot.armFlipper.stop();

            // a/b grip/ungrip fully
            if(gamepad2.a && !gamepad2.start && !gamepad2.back)
                robot.gripper.gripFully();
            else if(gamepad2.b && !gamepad2.start && !gamepad2.back)
                robot.gripper.ungripFully();
            else if(singleDriverMode && gamepad1.a && !gamepad1.start && !gamepad1.back)
                robot.gripper.gripFully();
            else if(singleDriverMode && gamepad1.b && !gamepad1.start && !gamepad1.back)
                robot.gripper.ungripFully();

            // x/y rotate
            // short press moves at half speed, after half a second it starts moving at full speed
            if(gamepad2.y || (singleDriverMode && gamepad1.y)) {
                y.down();
                if(y.getTimeDown() < 500) // short press
                    robot.rotator.rotate(.5); // just move a little bit (small adjustments)
                else // long press
                    robot.rotator.rotate(1); // move as fast as possible
            } else if(gamepad2.x || (singleDriverMode && gamepad1.x)) {
                x.down();
                if(x.getTimeDown() < 500)
                    robot.rotator.unrotate(.5);
                else
                    robot.rotator.unrotate(1);
            } else {
                x.up();
                y.up();
                robot.rotator.stop();
            }

            // gamepad1 a/b control launcher
            if(gamepad1.a && !singleDriverMode && !gamepad1.start && !gamepad1.back)
                robot.launcher.launch();
            else if(gamepad1.b && !singleDriverMode && !gamepad1.start && !gamepad1.back)
                robot.launcher.reset();

            telemetry.addData("Single driver mode", singleDriverMode);
            telemetry.addLine("\nintake: triggers (raise/lower intake: bumpers)");
            telemetry.addLine("arm: joysticks or d-pad");
            telemetry.addLine("gripper: a/b");
            telemetry.addLine("rotator: x/y");
            telemetry.addLine("launcher: gamepad1 a/b");
            telemetry.addData("\nGamepad1 left_stick_y", gamepad1.left_stick_y);
            telemetry.addData("Average wheel power", robot.drive.getAverageDriveMotorPower());
            telemetry.addData("\nIntake power", robot.intake.getPower());
            telemetry.addData("Arm flipper power", robot.armFlipper.getPower());
            telemetry.addData("Arm lifter powers", robot.armRaisers.getPower());
            telemetry.addData("Rotator power", "%.2f", robot.rotator.getPower());
            telemetry.addData("Gripper psn", "%s (%.3f)", robot.gripper.getStatus(), robot.gripper.getPosition());
            telemetry.addData("Intake psn", "%s (%.1f)", robot.intake.getStatus(), robot.intake.lowerer.getPosition());
            telemetry.addData("Launcher psn", "%.2f", robot.launcher.getPosition());
            telemetry.update();
        }
    }
}
