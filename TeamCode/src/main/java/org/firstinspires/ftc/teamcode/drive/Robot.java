package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public SampleMecanumDrive drive;
    public Intake intake;
    public ArmFlipper armFlipper;
    public ArmRaisers armRaisers;
    public Gripper gripper;
    public Rotator rotator;
    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap, "motor0");
        armFlipper = new ArmFlipper(hardwareMap, "motor1");
        armRaisers = new ArmRaisers(hardwareMap, "motor3", "motor2");
        gripper = new Gripper(hardwareMap, "servo0");
        rotator = new Rotator(hardwareMap, "servo1");
    }
}
