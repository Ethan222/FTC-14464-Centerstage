package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Generic motor class that controls one motor
@Config
public class Motor {
    public enum Status {
        DOWN, UP, UNSURE, MOVING_DOWN, MOVING_UP
    }
    protected final DcMotorEx motor;
    public static double DEFAULT_ACCELERATION = .01;
    private boolean usingEncoder;
    private int downPosition, upPosition;
    private boolean holding;
    private int holdPosition;
    public static int HOLD_PRECISION = 5;

    // constructors
    public Motor(HardwareMap hm, String name, boolean usingEncoder, int downPsn, int upPsn) {
        motor = hm.get(DcMotorEx.class, name); // get motor from the hardwareMap
        downPosition = downPsn;
        upPosition = upPsn;
        if(usingEncoder)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public Motor(HardwareMap hm, String name, boolean usingEncoder) { this(hm, name, usingEncoder, 0, 0); }
    public Motor(HardwareMap hm, String name) {
        this(hm, name, false);
    }
    public void setPower(double power) {
            motor.setPower(power);
    }
    public void changePower(double power) { setPower(getPower() + power); }
    public void stop() { setPower(0); }
    public double getPower() { return motor.getPower(); }
    public String getPowerAsString() {
        return String.format("%.1f", getPower());
    }

    // acceleration
    public void acceleratePositive(double acceleration) {
        if(getPower() < 1)
            changePower(acceleration);
    }
    public void acceleratePositive() {
        acceleratePositive(DEFAULT_ACCELERATION);
    }
    public void accelerateNegative(double acceleration) {
        if(getPower() > -1)
            changePower(-acceleration);
    }
    public void accelerateNegative() {
        accelerateNegative(DEFAULT_ACCELERATION);
    }
    public void decelerate(double deceleration) {
        if(getPower() > 0)
            accelerateNegative(deceleration);
        else if(getPower() < 0)
            acceleratePositive(deceleration);
    }
    public void decelerate() {
        decelerate(DEFAULT_ACCELERATION);
    }

    // encoders
    public void runUsingEncoder() {
        usingEncoder = true;
    }
    public void runWithoutEncoder() {
        usingEncoder = false;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public boolean isUsingEncoder() {
        return usingEncoder;
    }
    public int getPosition() {
        return motor.getCurrentPosition();
    }
    public boolean isIdle() {
        return !motor.isBusy();
    }

    public void goToPosition(int position, double power) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(position > getPosition())
            setPower(power);
        else
            setPower(-power);
    }
    public void goToPosition(int position) { goToPosition(position, 1); }
    public void goToUpPosition() {
        goToPosition(upPosition);
    }
    public void goToDownPosition() {
        goToPosition(downPosition);
    }
    public void setDownPosition() {
        downPosition = getPosition();
    }
    public void setUpPosition() {
        upPosition = getPosition();
    }
    public double getDownPosition() { return downPosition; }
    public double getUpPosition() { return upPosition; }

    // hold at the current position
    public void hold() {
        if(!holding) {
            holdPosition = getPosition();
            holding = true;
        } else if (Math.abs(getPosition() - holdPosition) > HOLD_PRECISION) {
            goToPosition(holdPosition);
        } else
            decelerate();
    }
    public void stopHolding() { holding = false; }
    public boolean isHolding() { return holding; }
    public int getHoldPosition() { return holdPosition; }

    public Status getStatus() {
        double power = getPower();
        double position = getPosition();
        if(power > .4)
            return Status.MOVING_UP;
        else if(power < -.4)
            return Status.MOVING_DOWN;
        else if(Math.abs(position - downPosition) < 50)
            return Status.DOWN;
        else if(Math.abs(position - upPosition) < 50)
            return Status.UP;
        else
            return Status.UNSURE;
    }
}
