package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.MecanumDrive;

public class Robot {
    public MecanumDrive drive;
    public Intake intake;
    public Claw claw1, claw2;
    public Outtake outtake;
    public HangSubsystem hangSubsystem;
    public AutoClaw autoClaw;
    public DroneLauncher launcher;
    public Robot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap, "intake", "servo4");
        claw1 = new Claw(hardwareMap, "servo1", .56, .78);
        claw2 = new Claw(hardwareMap, "servo2", .8, 1);
        outtake = new Outtake(hardwareMap, "armMotor", "outtakeRotator");
        hangSubsystem = new HangSubsystem(hardwareMap, new String[]{"hangMotor", "hang2"}, "servo5");
        autoClaw = new AutoClaw(hardwareMap, "droneLauncher");
        launcher = new DroneLauncher(hardwareMap, "launcherRotator", "servo3");
    }
}
