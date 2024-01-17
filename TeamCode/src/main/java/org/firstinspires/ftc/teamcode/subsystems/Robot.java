package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.SampleMecanumDrive;

public class Robot {
    public SampleMecanumDrive drive;
    public Intake intake;
    public Claw claw1, claw2;
    public Rotator rotator;
    public OuttakeRaiser outtakeRaiser;
    public HangMotor hangMotor;
    public AutoClaw autoClaw;
    public DroneLauncher launcher;
    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap, "intake", "servo4");
        claw1 = new Claw(hardwareMap, "servo1", .39, .64);
        claw2 = new Claw(hardwareMap, "servo2", .7, 1);
        rotator = new Rotator(hardwareMap, "servo0", .52, .08);
        outtakeRaiser = new OuttakeRaiser(hardwareMap, "armMotor");
        hangMotor = new HangMotor(hardwareMap, "hangMotor");
        autoClaw = new AutoClaw(hardwareMap, "servo3", .06, .54);
//        launcher = new DroneLauncher(hardwareMap, "servo5");
    }
}
