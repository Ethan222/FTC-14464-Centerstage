package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class CustomRobot {
    public SampleMecanumDrive drive;
    public Intake intake;
    public ArmFlipper armFlipper;
    public ArmLifters armLifters;
    public Claw claw;
    public CustomRobot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap, "motor0");
        armFlipper = new ArmFlipper(hardwareMap, "motor1");
        armLifters = new ArmLifters(hardwareMap, "motor3", "motor2");
        claw = new Claw(hardwareMap, "servo0", "servo1");
    }
}
