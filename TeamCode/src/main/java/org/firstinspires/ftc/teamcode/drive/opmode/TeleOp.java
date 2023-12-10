package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        Robot robot = new Robot(hardwareMap);
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean singleDriverMode = true;

        while(!isStarted() && !isStopRequested()) {
            telemetry.addLine("Initialized");
            telemetry.addData("Single driver mode", singleDriverMode);
            telemetry.update();

            if(gamepad1.back && gamepad1.a)
                singleDriverMode = true;
            else if(gamepad1.back && gamepad1.b)
                singleDriverMode = false;
            else if(gamepad2.back && gamepad2.b)
                singleDriverMode = false;
        }
        //waitForStart();

        while (!isStopRequested() && !(gamepad1.start && gamepad1.x) && !(gamepad2.start && gamepad2.x)) {
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
                robot.armRaisers.setPower(-gamepad2.left_stick_y);
            else if(gamepad2.dpad_up)
                robot.armRaisers.up();
            else if(gamepad2.dpad_down)
                robot.armRaisers.down();
            else if(singleDriverMode && gamepad1.dpad_up)
                robot.armRaisers.up();
            else if(singleDriverMode && gamepad1.dpad_down)
                robot.armRaisers.down();
            else
                robot.armRaisers.stop();

            // right stick x or d pad left/right controls arm flipper
            if(gamepad2.right_stick_x != 0)
                robot.armFlipper.flip(gamepad2.right_stick_x);
            else if(gamepad2.dpad_right)
                robot.armFlipper.flip();
            else if(gamepad2.dpad_left)
                robot.armFlipper.unflip();
            else if(singleDriverMode && gamepad1.dpad_right)
                robot.armFlipper.flip();
            else if(singleDriverMode && gamepad1.dpad_left)
                robot.armFlipper.unflip();
            else
                robot.armFlipper.stop();

            // a/b grip/ungrip fully, bumpers controls gripper incrementally
            if(gamepad2.a && !gamepad2.start)
                robot.claw.gripFully();
            else if(gamepad2.b && !gamepad2.start && !gamepad2.back)
                robot.claw.ungripFully();
            else if(gamepad2.right_bumper)
                robot.claw.gripIncrementally();
            else if(gamepad2.left_bumper)
                robot.claw.ungripIncrementally();
            else if(singleDriverMode && gamepad1.a && !gamepad1.start && !gamepad1.back)
                robot.claw.gripFully();
            else if(singleDriverMode && gamepad1.b && !gamepad1.start && !gamepad1.back)
                robot.claw.ungripFully();
            else if(singleDriverMode && gamepad1.right_bumper)
                robot.claw.gripIncrementally();
            else if(singleDriverMode && gamepad1.left_bumper)
                robot.claw.ungripIncrementally();

            // x/y rotate incrementally
            if(gamepad2.x)
                robot.claw.rotateIncrementally();
            else if(gamepad2.y)
                robot.claw.unrotateIncrementally();
            else if(singleDriverMode && gamepad1.x)
                robot.claw.rotateIncrementally();
            else if(singleDriverMode && gamepad1.y)
                robot.claw.unrotateIncrementally();

            telemetry.addData("Single driver mode", singleDriverMode);
            telemetry.addLine("intake: triggers");
            telemetry.addLine("gripper: bumpers or a/b");
            telemetry.addLine("arm: left_stick or d-pad");
            telemetry.addLine("rotator: x/y");
            telemetry.addData("Intake power", robot.intake.getPower());
            telemetry.addData("Arm flipper power", robot.armFlipper.getPower());
            telemetry.addData("Arm lifter powers", robot.armRaisers.getPower());
            telemetry.addLine(String.format("Gripper psn: %s (%s)",
                    robot.claw.gripper.getRoundedPsn(), robot.claw.gripperStatus()));
            telemetry.addLine(String.format("Rotator psn: %s (%s)",
                    robot.claw.rotator.getRoundedPsn(), robot.claw.rotatorStatus()));
            telemetry.update();
        }
    }
}
