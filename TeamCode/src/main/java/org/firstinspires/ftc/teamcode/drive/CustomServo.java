package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CustomServo {
    private Servo servo;
    private double leftPosition, rightPosition;
    public CustomServo(HardwareMap hm, String name, double leftPsn, double rightPsn) {
        servo = hm.get(Servo.class, name);
        leftPosition = leftPsn;
        rightPosition = rightPsn;
    }
    public CustomServo(HardwareMap hm, String name) {
        this(hm, name, 0, 1);
    }
    public void setPosition(double psn) {
        if(psn < leftPosition)
            servo.setPosition(leftPosition);
        else if(psn > rightPosition)
            servo.setPosition(rightPosition);
        else
            servo.setPosition(psn);
    }
    public double getPosition() {
        return servo.getPosition();
    }
    public void changePosition(double chg) {
        setPosition(getPosition() + chg);
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
