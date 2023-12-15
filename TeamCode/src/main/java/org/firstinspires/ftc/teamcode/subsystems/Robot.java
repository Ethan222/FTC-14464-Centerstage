package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class Robot {
    public SampleMecanumDrive drive;
    public Intake intake;
    public ArmFlipper armFlipper;
    public ArmRaisers armRaisers;
    public Gripper gripper;
    public Rotator rotator;
    public DroneLauncher launcher;
    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap, "motor0");
        armFlipper = new ArmFlipper(hardwareMap, "motor1");
        armRaisers = new ArmRaisers(hardwareMap, "motor3", "motor2");
        gripper = new Gripper(hardwareMap, "servo0");
        rotator = new Rotator(hardwareMap, "servo1");
        launcher = new DroneLauncher(hardwareMap, "servo3");
    }
}
