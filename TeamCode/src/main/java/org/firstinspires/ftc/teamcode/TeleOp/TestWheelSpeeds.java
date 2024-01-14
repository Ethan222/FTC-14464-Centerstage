package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test Wheel Speeds", group = "test")
public class TestWheelSpeeds extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        Robot robot = new Robot(hardwareMap);
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Initialized");
        telemetry.update();

        while(opModeInInit() && !(gamepad1.start && gamepad1.x) && !(gamepad2.start && gamepad2.x));

        double coeff = 1, exp = 1;

        while (opModeIsActive() && !(gamepad1.start && gamepad1.x) && !(gamepad2.start && gamepad2.x)) {
            if(gamepad1.right_trigger > .1 && exp < 7)
                exp += .4;
            else if(gamepad1.left_trigger > .1 && exp > .2)
                exp -= .4;

            if(gamepad1.right_bumper && coeff < 1)
                coeff += .1;
            else if(gamepad1.left_bumper && coeff > .1)
                coeff -= .1;

            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            coeff * -Math.pow(gamepad1.left_stick_y, exp),
                            coeff * -Math.pow(gamepad1.left_stick_x, exp),
                            coeff * -Math.pow(gamepad1.right_stick_x, exp)
                    )
            );
            robot.drive.update();

            telemetry.addData("Coefficient (change with bumpers)", coeff);
            telemetry.addData("Exponent (change with triggers)", exp);
            telemetry.addData("\nGamepad1 left_stick_y", gamepad1.left_stick_y);
            telemetry.addData("Average wheel power", robot.drive.getAverageDriveMotorPower());
            telemetry.update();
        }
    }
}
