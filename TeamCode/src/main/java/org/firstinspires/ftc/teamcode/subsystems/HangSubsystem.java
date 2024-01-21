package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

// controls motor that hangs
public class HangSubsystem extends Motor {     // inherits from Motor
    public CustomServo rotator;
    public HangSubsystem(HardwareMap hm, String motorName, String servoName) {             // constructor
        super(hm, motorName, true, 0, 12900);      // calls parent constructor with default speed and max speed
        rotator = new CustomServo(hm, servoName, .1, .6);
    }
    public void up(double power) {
        setPower(power);
    }
    public void down(double power) { setPower(-power); }
    public void up() {
        up(1);
    }
    public void down() {
        down(1);
    }
    public void rotateUp(double increment) {
        rotator.changePosition(-increment);
    }
    public void rotateUp() {
        rotator.goToLeft();
    }
    public void rotateDown(double increment) {
        rotator.changePosition(increment);
    }
    public void rotateDown() {
        rotator.goToRight();
    }
    public String getRotatorStatus() {
        String status = rotator.getStatus();
        if(status.equals(CustomServo.Status.RIGHT.toString()))
            return "down";
        else if(status.equals(CustomServo.Status.LEFT.toString()))
            return "up";
        else
            return "in between";
    }
}
