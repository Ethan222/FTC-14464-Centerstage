package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Generic class to represent a servo
public class CustomServo {
    private Servo servo;                                // field representing the actual Servo object
    private double leftPosition, rightPosition;         // fields representing the farthest left and right the servo can go
    private double DEFAULT_INCREMENT = .01;
    public enum Status {
        LEFT, MIDDLE, RIGHT
    }
    public CustomServo(HardwareMap hm, String name, double leftPsn, double rightPsn, double increment) {
        servo = hm.get(Servo.class, name);
        leftPosition = leftPsn;
        rightPosition = rightPsn;
        if(increment != 0)
            DEFAULT_INCREMENT = increment;
    }
    public CustomServo(HardwareMap hm, String name, double left, double right) { this(hm, name, left, right, 0); }
    public CustomServo(HardwareMap hm, String name, double increment) { this(hm, name, 0, 1, increment); }
    public CustomServo(HardwareMap hm, String name) {   // constructor overload with default left and right positions
        this(hm, name, 0);
    }
    public void setPosition(double psn) {               // method to set the servo's position
        if(psn < leftPosition)
            servo.setPosition(leftPosition);
        else
            servo.setPosition(Math.min(psn, rightPosition));
    }
    public void changePosition(double chg) {        // method to change the servo's position by a given amount
        setPosition(getPosition() + chg);
    }
    public void increasePosition() { changePosition(DEFAULT_INCREMENT); }
    public void decreasePosition() { changePosition(-DEFAULT_INCREMENT); }
    public double getPosition() {
        return servo.getPosition();
    }
    public void goToLeft() {
        setPosition(leftPosition);
    }
    public void goToRight() {
        setPosition(rightPosition);
    }
    public String getStatus() {
        double psn = getPosition();
        if (Math.abs(psn - leftPosition) < .03) {
            return Status.LEFT.toString();
        } else if (Math.abs(psn - rightPosition) < .03) {
            return Status.RIGHT.toString();
        } else {
            return Status.MIDDLE.toString();
        }
    }
}
