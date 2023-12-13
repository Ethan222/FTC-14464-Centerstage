package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

// class to control the intake motor
public class Intake extends Motor { // inherits from parent class Motor
    public CustomServo lowerer;     // servo to raise/lower the intake
    private static final double UP_PSN = 1, DOWN_PSN = 0;   // up and down positions of the servo
    public Intake(HardwareMap hardwareMap, String name) {        // constructor
        super(hardwareMap, name, .8, .9);   // calls parent constructor
        lowerer = new CustomServo(hardwareMap, "servo2", DOWN_PSN, UP_PSN); // initializes lowerer servo
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
    public void lower() { // lower intake
        lowerer.setPosition(DOWN_PSN);
    }
    public void raise() { // raise intake
        lowerer.setPosition(UP_PSN);
    }
    // return the status of the intake (up or down)
    public String getStatus() {
        switch((int)lowerer.getPosition()) {
            case (int)UP_PSN:
                return "up";
            case (int)DOWN_PSN:
                return "down";
            default:
                return "partially down";
        }
    }
}
