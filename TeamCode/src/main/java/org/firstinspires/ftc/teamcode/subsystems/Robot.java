package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.SampleMecanumDrive;

public class Robot {
    public SampleMecanumDrive drive;
    public Intake intake;
    public ArmFlipper armFlipper;
    public ArmRaiser armRaiser;
    public Gripper gripper;
    public Rotator rotator;
    public DroneLauncher launcher;
    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap, "intake");
        //armFlipper = new ArmFlipper(hardwareMap, "motor1");
        armRaiser = new ArmRaiser(hardwareMap, "armMotor");
//        gripper = new Gripper(hardwareMap, "servo0");
//        rotator = new Rotator(hardwareMap, "servo1");
//        launcher = new DroneLauncher(hardwareMap, "servo3");
    }
}
