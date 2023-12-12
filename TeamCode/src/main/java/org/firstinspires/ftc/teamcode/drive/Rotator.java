package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

// controls the continuous servo that rotates the claw mechanism
public class Rotator {
    private CRServo rotator;                            // the actual continous servo
    private static final double DEFAULT_SPEED = 1;      // constant field to represent the default speed the servo should run at
    // constructor: initializes the servo and retrieves it from the hardware map
    public Rotator(HardwareMap hm, String name) {
        rotator = hm.get(CRServo.class, name);
    }
    public void setPower(double power) {
        rotator.setPower(power);
    }   // sets the power of the servo
    public void rotate(double power) {
        setPower(power);
    }             // rotates the claw
    public void rotate() {
        rotate(DEFAULT_SPEED);
    }
    public void unrotate(double power) {
        setPower(-power);
    }
    public void unrotate() {
        unrotate(DEFAULT_SPEED);
    }
    public void stop() {
        rotator.setPower(0);
    }
    public double getPower() {
        return rotator.getPower();
    }
}
