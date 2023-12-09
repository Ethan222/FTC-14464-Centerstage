package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class CustomRobot {
    public SampleMecanumDrive drive;
    public Intake intake;
    public ArmTurner armTurner;
    public ArmLifters armLifters;
    public Claw claw;
    public CustomRobot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap, "motor0");
        armTurner = new ArmTurner(hardwareMap, "motor2");
        armLifters = new ArmLifters(hardwareMap, "motor1", "motor3");
        claw = new Claw(hardwareMap, "servo1", "servo0");
    }
}
