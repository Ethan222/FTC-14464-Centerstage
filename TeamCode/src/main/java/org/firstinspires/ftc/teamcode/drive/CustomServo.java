package org.firstinspires.ftc.teamcode.drive;

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
}
