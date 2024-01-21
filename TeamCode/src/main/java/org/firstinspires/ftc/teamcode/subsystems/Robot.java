package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.SampleMecanumDrive;

public class Robot {
    public SampleMecanumDrive drive;
    public Intake intake;
    public Claw claw1, claw2;
    public Outtake outtake;
    public HangSubsystem hangSubsystem;
    public AutoClaw autoClaw;
    public DroneLauncher launcher;
    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap, "intake", "servo4");
        claw1 = new Claw(hardwareMap, "servo1", .39, .64);
        claw2 = new Claw(hardwareMap, "servo2", .7, 1);
        outtake = new Outtake(hardwareMap, "armMotor", "servo0");
        hangSubsystem = new HangSubsystem(hardwareMap, "hangMotor", "servo5");
        autoClaw = new AutoClaw(hardwareMap, "servo3", .10, .4);
        launcher = new DroneLauncher(hardwareMap, "droneLauncher");
    }
}
