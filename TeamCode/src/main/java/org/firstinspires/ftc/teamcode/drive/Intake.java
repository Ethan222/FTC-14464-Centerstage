package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

// controls the intake motor
public class Intake extends CustomMotor {
    public Intake(HardwareMap hardwareMap, String name) {
        super(hardwareMap, name, .5, .7);
    }
    // positive intakes, negative outtakes
    public void in(double power) {
        setPower(power);
    }
    public void in() { in(DEFAULT_SPEED); }
    public void out(double power) { setPower(-power); }
    public void out() { out(DEFAULT_SPEED); }
}
