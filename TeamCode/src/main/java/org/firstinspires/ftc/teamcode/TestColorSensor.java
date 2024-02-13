package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(group = "test")
public class TestColorSensor extends LinearOpMode {
    private ColorSensor colorSensor;

    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        colorSensor = new ColorSensor(hardwareMap, "colorSensor");

        telemetry.addLine("Initialized");
        telemetry.update();

        while (opModeInInit());

        while(opModeIsActive()) {
            telemetry.addData("Red", colorSensor.colorSensor.red());
            telemetry.addData("Green", colorSensor.colorSensor.green());
            telemetry.addData("Blue", colorSensor.colorSensor.blue());
            telemetry.addData("Alpha", colorSensor.colorSensor.alpha());
            telemetry.addData("\nColor", colorSensor.getColor());
            telemetry.update();
        }
    }
}
