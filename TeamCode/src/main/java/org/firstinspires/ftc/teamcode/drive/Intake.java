package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

// class to control the intake motor
public class Intake extends Motor { // inherits from parent class Motor
    public Intake(HardwareMap hardwareMap, String name) {        // constructor
        super(hardwareMap, name, .7, .8);   // calls parent constructor
    }
    // positive intakes
    public void in(double power) {
        setPower(power);
    }
    // function overload: if no power given, use the default speed
    public void in() { in(DEFAULT_SPEED); }
    // negative outtakes
    public void out(double power) { setPower(-power); }
    // function overload: if no power given, use the default speed
    public void out() { out(DEFAULT_SPEED); }
}
