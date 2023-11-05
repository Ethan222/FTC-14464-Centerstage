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

            // use the right stick or d-pad to control the arm
            if(gamepad2.right_stick_y != 0)
                drive.setArmPower(gamepad2.right_stick_y);
            else if(gamepad2.dpad_up)
                drive.armUp();
            else if(gamepad2.dpad_down)
                drive.armDown();
            else if(gamepad1.right_stick_button)
                drive.setArmPower(gamepad1.right_stick_y);
            else if(gamepad1.dpad_up)
                drive.armUp();
            else if(gamepad1.dpad_down)
                drive.armDown();
            else
                drive.stopArm();

            telemetry.addLine("Use the triggers to control the intake and the right stick y or d-pad to control the arm");
            telemetry.addData("Intake power", drive.getIntakePower());
            telemetry.addData("Arm power", drive.getArmPower());
            telemetry.update();
        }
    }
}
