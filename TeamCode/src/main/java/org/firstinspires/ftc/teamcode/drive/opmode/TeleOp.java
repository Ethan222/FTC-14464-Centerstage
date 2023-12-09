package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.CustomRobot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        CustomRobot robot = new CustomRobot(hardwareMap);
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean singleDriverMode = true;

        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();

        while (!isStopRequested()) {
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            robot.drive.update();

            if(gamepad1.back && gamepad1.a)
                singleDriverMode = true;
            else if(gamepad1.back && gamepad1.b)
                singleDriverMode = false;
            else if(gamepad2.back && gamepad2.b)
                singleDriverMode = false;

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

            // use the left stick y or d-pad up/down to control the arm lifter
            if(gamepad2.left_stick_y != 0)
                robot.armLifters.setPower(gamepad2.left_stick_y);
            else if(gamepad2.dpad_up)
                robot.armLifters.up();
            else if(gamepad2.dpad_down)
                robot.armLifters.down();
            else if(singleDriverMode && gamepad1.dpad_up)
                robot.armLifters.up();
            else if(singleDriverMode && gamepad1.dpad_down)
                robot.armLifters.down();
            else
                robot.armLifters.stop();

            // left stick x or d pad left/right controls arm turner
            if(gamepad2.left_stick_x != 0)
                robot.armTurner.turn(gamepad2.left_stick_x);
            else if(gamepad2.dpad_left)
                robot.armTurner.turn();
            else if(gamepad2.dpad_right)
                robot.armTurner.unturn();
            else if(singleDriverMode && gamepad1.dpad_left)
                robot.armTurner.turn();
            else if(singleDriverMode && gamepad1.dpad_right)
                robot.armTurner.unturn();
            else
                robot.armTurner.stop();

            // a/b grip/ungrip fully, bumpers controls gripper incrementally
            if(gamepad2.a)
                robot.claw.gripFully();
            else if(gamepad2.b)
                robot.claw.ungripFully();
            else if(gamepad2.left_bumper)
                robot.claw.gripIncrementally();
            else if(gamepad2.right_bumper)
                robot.claw.ungripIncrementally();
            else if(singleDriverMode && gamepad1.a)
                robot.claw.gripFully();
            else if(singleDriverMode && gamepad1.b)
                robot.claw.ungripFully();
            else if(singleDriverMode && gamepad1.left_bumper)
                robot.claw.gripIncrementally();
            else if(singleDriverMode && gamepad1.right_bumper)
                robot.claw.ungripIncrementally();

            // right stick x rotates incrementally, x/y rotate fully
            if(gamepad2.y)
                robot.claw.rotateFully();
            else if(gamepad2.x)
                robot.claw.unrotateFully();
            else if(gamepad2.right_stick_x != 0)
                robot.claw.rotateIncrementally(gamepad2.right_stick_x);
            else if(singleDriverMode && gamepad1.y)
                robot.claw.rotateIncrementally();
            else if(singleDriverMode && gamepad1.x)
                robot.claw.unrotateIncrementally();

            telemetry.addData("Single Driver Mode", singleDriverMode);
            telemetry.addLine("Gamepad 2: triggers control intake, bumpers & a/b control gripper");
            telemetry.addLine("left_stick or d-pad controls arm raising/lowering & turning");
            telemetry.addLine("right_stick_x or x/y control rotator");
            telemetry.addData("Intake power", robot.intake.getPower());
            telemetry.addData("Arm turner power", robot.armTurner.getPower());
            telemetry.addData("Arm lifter powers", robot.armLifters.getPower());
            telemetry.addData("Gripper psn", robot.claw.gripper.getPosition());
            telemetry.addData("Rotator psn", robot.claw.rotator.getPosition());
            telemetry.update();
        }
    }
}
