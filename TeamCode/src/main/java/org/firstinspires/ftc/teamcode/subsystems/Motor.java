package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Generic motor class that controls one motor
@Config
public class Motor {
    public enum State {
        DOWN, UP, UNSURE
    }
    protected final DcMotorEx motor;
    public static double DEFAULT_ACCELERATION = .1;
    private boolean usingEncoder;
    private int downPosition, upPosition;
    private boolean holding;
    private int holdPosition;
    public static int HOLD_PRECISION = 5;
    private Telemetry.Item telemetry;
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
        updateTelemetry();
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

    public State getState() {
        double position = getPosition();
        if(Math.abs(position - downPosition) < 50)
            return State.DOWN;
        else if(Math.abs(position - upPosition) < 50)
            return State.UP;
        else
            return State.UNSURE;
    }

    public void setTelemetry(Telemetry.Item item) {
        telemetry = item;
    }
    public void updateTelemetry() {
        telemetry.setValue("%s %s [%s]", getState(), isUsingEncoder() ? String.format("(%d)", getPosition()) : null, getPowerAsString());
    }
}
