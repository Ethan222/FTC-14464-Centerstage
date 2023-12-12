package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Generic class to represent a servo
public class CustomServo {
    private Servo servo;                                // field representing the actual Servo object
    private double leftPosition, rightPosition;         // fields representing the farthest left and right the servo can go
    public CustomServo(HardwareMap hm, String name, double leftPsn, double rightPsn) {
        servo = hm.get(Servo.class, name);
        leftPosition = leftPsn;
        rightPosition = rightPsn;
    }       // constructor
    public CustomServo(HardwareMap hm, String name) {   // constructor overload with default left and right positions
        this(hm, name, 0, 1);
    }
    public void setPosition(double psn) {               // method to set the servo's position
        if(psn < leftPosition)                          // checks to see if the position is valid
            servo.setPosition(leftPosition);            // i.e., that it's not less than the leftmost position
        else if(psn > rightPosition)                    // or greater than the rightmost position
            servo.setPosition(rightPosition);           // if it's outside that range, it moves as far as it can
        else                                            // but if it is valid,
            servo.setPosition(psn);                     // it moves to that position
    }
    public void changePosition(double chg) {        // method to change the servo's position by a given amount
        setPosition(getPosition() + chg);
    }
    public double getPosition() {
        return servo.getPosition();
    }
    public void goToLeft() {
        setPosition(leftPosition);
    }
    public void goToRight() {
        setPosition(rightPosition);
    }
    @SuppressLint("DefaultLocale")
    public String getRoundedPsn() {
        return String.format("%.3f", getPosition());
    }
}
