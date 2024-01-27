package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// controls 2 motors that hang
public class HangSubsystem {
    public final static String DOWN = "DOWN", UP = "UP", IN_BETWEEN = "IN BETWEEN";
    private final Motor[] motors;
    public CustomServo rotator;
    public HangSubsystem(HardwareMap hm, String[] motorNames, String servoName) {
        motors = new Motor[]{
                new Motor(hm, motorNames[0]),
                new Motor(hm, motorNames[1])
        };
        motors[1].motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rotator = new CustomServo(hm, servoName, .5, 1);
    }
    public void setPowers(double power) {
        for(Motor motor : motors)
            motor.setPower(power);
    }
    public void up(double power) {
        setPowers(power);
    }
    public void down(double power) {
        setPowers(-power);
    }
    public void up() {
        up(1);
    }
    public void down() {
        down(1);
    }
    public String getPowersAsString() {
        return motors[0].getPowerAsString() + ", " + motors[1].getPowerAsString();
    }
    public void runUsingEncoder() {
        for(Motor motor : motors)
            motor.runUsingEncoder();
    }
    public void runWithoutEncoder() {
        for(Motor motor : motors)
            motor.runWithoutEncoder();
    }
    public boolean isUsingEncoder() {
        return motors[0].isUsingEncoder() && motors[1].isUsingEncoder();
    }
    public String getPositionsAsStrings() {
        return motors[0].getPosition() + ", " + motors[1].getPosition();
    }
    public boolean isIdle() {
        return motors[0].isIdle() && motors[1].isIdle();
    }



    public void rotateUp(double increment) {
        rotator.changePosition(increment);
    }
    public void rotateUp() {
        rotator.goToRight();
    }
    public void rotateDown(double increment) {
        rotator.changePosition(-increment);
    }
    public void rotateDown() {
        rotator.goToLeft();
    }
    public String getRotatorStatus() {
        String status = rotator.getStatus();
        if(status.equals(CustomServo.LEFT))
            return DOWN;
        else if(status.equals(CustomServo.RIGHT))
            return UP;
        else
            return IN_BETWEEN;
    }
}
