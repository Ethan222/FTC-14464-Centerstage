package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@Disabled
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test Wheel Speeds", group = "test")
public class TestWheelSpeeds extends LinearOpMode {
    public static double coeff = 1, exp = 1;
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

        while (opModeIsActive() && !(gamepad1.start && gamepad1.x) && !(gamepad2.start && gamepad2.x)) {
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            coeff * -Math.pow(gamepad1.left_stick_y, exp),
                            coeff * -Math.pow(gamepad1.left_stick_x, exp),
                            coeff * -Math.pow(gamepad1.right_stick_x, exp)
                    )
            );
            robot.drive.update();

            telemetry.addData("Coefficient", coeff);
            telemetry.addData("Exponent", exp);
            telemetry.addData("\nGamepad1 left_stick_y", gamepad1.left_stick_y);
            telemetry.addData("Average wheel power", robot.drive.getAverageDriveMotorPower());
            telemetry.update();
        }
    }
}
