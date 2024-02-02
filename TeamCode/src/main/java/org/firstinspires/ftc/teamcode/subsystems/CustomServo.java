package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Generic class to represent a servo
public class CustomServo {
    public final static String LEFT = "LEFT", MIDDLE = "MIDDLE", RIGHT = "RIGHT";
    private final Servo servo;                                // field representing the actual Servo object
    private final double LEFT_POSITION, RIGHT_POSITION;         // fields representing the farthest left and right the servo can go
    private double DEFAULT_INCREMENT = .01;
    private Telemetry.Item telemetry;
    public CustomServo(HardwareMap hm, String name, double leftPsn, double rightPsn, double increment) {
        servo = hm.get(Servo.class, name);
        LEFT_POSITION = leftPsn;
        RIGHT_POSITION = rightPsn;
        if(increment != 0)
            DEFAULT_INCREMENT = increment;
    }
    public CustomServo(HardwareMap hm, String name, double left, double right) { this(hm, name, left, right, 0); }
    public CustomServo(HardwareMap hm, String name, double increment) { this(hm, name, 0, 1, increment); }
    public CustomServo(HardwareMap hm, String name) {   // constructor overload with default left and right positions
        this(hm, name, 0);
    }
    public void setPosition(double psn) {               // method to set the servo's position
        if(psn < LEFT_POSITION)
            servo.setPosition(LEFT_POSITION);
        else
            servo.setPosition(Math.min(psn, RIGHT_POSITION));
        updateTelemetry();
    }
    public void changePosition(double chg) {        // method to change the servo's position by a given amount
        setPosition(getPosition() + chg);
    }
    public void increasePosition() { changePosition(DEFAULT_INCREMENT); }
    public void decreasePosition() { changePosition(-DEFAULT_INCREMENT); }
    public double getPosition() {
        return servo.getPosition();
    }
    public double getLeftPosition() { return LEFT_POSITION; }
    public double getRightPosition() { return RIGHT_POSITION; }
    public void goToLeft() {
        setPosition(LEFT_POSITION);
    }
    public void goToRight() {
        setPosition(RIGHT_POSITION);
    }
    public String getState() {
        double psn = getPosition();
        if (Math.abs(psn - LEFT_POSITION) < .05) {
            return LEFT;
        } else if (Math.abs(psn - RIGHT_POSITION) < .05) {
            return RIGHT;
        } else {
            return MIDDLE;
        }
    }
    public double getPercent() {
        return (getPosition() - getLeftPosition()) / (getRightPosition() - getLeftPosition());
    }

    public void setTelemetry(Telemetry.Item item) {
        telemetry = item;
    }
    public void updateTelemetry() {
        telemetry.setValue("%s (%.2f)", getState(), getPosition());
    }
}
