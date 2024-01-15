package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.SampleMecanumDrive;

public class Robot {
    public SampleMecanumDrive drive;
    public Intake intake;
    public Gripper gripper1, gripper2;
    public Rotator rotator;
    public OuttakeRaiser outtakeRaiser;
    public HangMotor hangMotor;
    public AutoClaw autoClaw;
    public DroneLauncher launcher;
    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap, "intake", "servo4");
        gripper1 = new Gripper(hardwareMap, "servo1", .39, .64);
        gripper2 = new Gripper(hardwareMap, "servo2", .7, 1);
        rotator = new Rotator(hardwareMap, "servo0", .52, .12);
        outtakeRaiser = new OuttakeRaiser(hardwareMap, "armMotor");
        hangMotor = new HangMotor(hardwareMap, "hangMotor");
        autoClaw = new AutoClaw(hardwareMap, "servo3", .06, .54);
//        launcher = new DroneLauncher(hardwareMap, "servo5");
    }
}
