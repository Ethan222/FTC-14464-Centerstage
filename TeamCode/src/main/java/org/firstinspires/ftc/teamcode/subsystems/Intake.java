package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

// class to control the intake mechanism
public class Intake extends Motor { // inherits from parent class Motor
    public CustomServo lowerer;     // servo to raise/lower the intake
    private static final double UP_PSN = 1, DOWN_PSN = 0;   // up and down positions of the servo
    public Intake(HardwareMap hardwareMap, String name) {        // constructor
        super(hardwareMap, name, 1, 1);   // calls parent constructor to initialize intake motor
        lowerer = new CustomServo(hardwareMap, "intakeRaiser", DOWN_PSN, UP_PSN); // initializes lowerer servo
    }
    public void in(double power) {
        setPower(power);
    } // positive intakes
    public void in() { in(DEFAULT_SPEED); }     // function overload: if no power given, use the default speed
    public void out(double power) { setPower(-power); } // negative outtakes
    public void out() { out(DEFAULT_SPEED); }   // function overload: if no power given, use the default speed
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
