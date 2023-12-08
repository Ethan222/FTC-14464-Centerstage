package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            // use the triggers to control the intake - right trigger intakes, left trigger outtakes
            if(gamepad2.right_trigger > 0)
                drive.intake(gamepad2.right_trigger);
            else if(gamepad2.left_trigger > 0)
                drive.outtake(gamepad2.left_trigger);
            else if(gamepad1.right_trigger > 0)
                drive.intake(gamepad1.right_trigger);
            else if(gamepad1.left_trigger > 0)
                drive.outtake(gamepad1.left_trigger);
            else
                drive.stopIntake();
            /*
            // bumpers control gripper
            if(gamepad1.left_bumper)
                drive.gripper.setPower(.8);
            else if(gamepad1.right_bumper)
                drive.gripper.setPower(-.8);
            else
                drive.gripper.setPower(0);
            */
            // use the right stick or d-pad to control the arm lifter
            if(gamepad2.right_stick_y != 0)
                drive.setArmLifterPower(gamepad2.right_stick_y);
            else if(gamepad2.dpad_up)
                drive.armUp();
            else if(gamepad2.dpad_down)
                drive.armDown();
            else if(gamepad1.right_stick_button)
                drive.setArmLifterPower(gamepad1.right_stick_y);
            else if(gamepad1.dpad_up)
                drive.armUp();
            else if(gamepad1.dpad_down)
                drive.armDown();
            else
                drive.stopArmLifter();

            // d pad left/right controls arm turner
            if(gamepad1.dpad_left)
                drive.armTurn();
            else if(gamepad1.dpad_right)
                drive.armUnturn();
            else
                drive.stopArmTurner();

            // a/b controls gripper
            if(gamepad1.a)
                drive.gripper.setPosition(drive.gripper.getPosition() + .01);
            else if(gamepad1.b)
                drive.gripper.setPosition(drive.gripper.getPosition() - .01);

            // x/y control rotator
            double chg;
            if(gamepad1.x)
                chg = -.005;
            else if(gamepad1.y)
                chg = .005;
            else
                chg = 0;
            if(chg != 0) {
                drive.rotator.setPosition(drive.rotator.getPosition() + chg);
                sleep(100);
            }

            telemetry.addLine("triggers control intake, d-pad up/down & left/right controls arm raising/lowering & turning");
            telemetry.addLine("a/b control gripper, x/y control rotator");
            telemetry.addLine(String.format("Intake power: %.3f", drive.getIntakePower()));
            //telemetry.addData("Gripper pwr", drive.gripper.getPower());
            double[] armLifterPowers = drive.getArmLifterPowers();
            telemetry.addLine(String.format("Arm lifter powers: [%.3f, %.3f]", armLifterPowers[0], armLifterPowers[1]));
            telemetry.addLine(String.format("Arm turner power: %.3f", drive.getArmTurnerPower()));
            telemetry.addLine(String.format("Gripper psn: %.3f", drive.gripper.getPosition()));
            telemetry.addLine(String.format("Rotator psn: %.3f", drive.rotator.getPosition()));
            telemetry.update();
        }
    }
}
