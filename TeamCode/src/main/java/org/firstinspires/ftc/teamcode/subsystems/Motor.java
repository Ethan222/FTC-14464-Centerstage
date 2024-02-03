package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

// Generic motor class that controls one motor
@Config
public class Motor {
    public enum State {
        DOWN, UP, UNSURE
    }
    protected final DcMotorEx motor;
    public static double DEFAULT_ACCELERATION = .6;
    private boolean usingEncoder;
    private int downPosition, upPosition;
    private boolean holding;
    private int holdPosition;
    public static int HOLD_PRECISION = 30;

    protected Telemetry.Item telemetry;
    private final ScheduledExecutorService executorService;
    // constructors
    public Motor(HardwareMap hm, String name, boolean usingEncoder, int downPsn, int upPsn) {
        motor = hm.get(DcMotorEx.class, name); // get motor from the hardwareMap
        downPosition = downPsn;
        upPosition = upPsn;
        if(usingEncoder)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        executorService = Executors.newSingleThreadScheduledExecutor();
    }
    public Motor(HardwareMap hm, String name, boolean usingEncoder) { this(hm, name, usingEncoder, 0, 0); }
    public Motor(HardwareMap hm, String name) {
        this(hm, name, false);
    }
    public void setPower(double power) {
        if(power > 1)
            motor.setPower(1);
        else
            motor.setPower(Math.max(power, -1));
        updateTelemetry();
    }
    public void changePower(double power) { setPower(getPower() + power); }
    public void stop() { setPower(0); }
    public double getPower() { return motor.getPower(); }
    @SuppressLint("DefaultLocale")
    public String getPowerAsString() {
        return String.format("%.1f", getPower());
    }

    // acceleration
    public void accelerateTo(double targetPower) {
        if(getPower() == targetPower)
            return;

        Runnable commandToRun;
        if(targetPower > getPower())
            commandToRun = this::acceleratePositive;
        else
            commandToRun = this::accelerateNegative;
        int n = (int)Math.ceil((targetPower - getPower()) / DEFAULT_ACCELERATION);
        for(int i = 0; i < n; i++) {
            executorService.schedule(commandToRun, i * 50L, TimeUnit.MILLISECONDS);
        }
    }
    public void accelerate(double acceleration) {
        changePower(acceleration);
    }
    public void acceleratePositive() {
        accelerate(DEFAULT_ACCELERATION);
    }
    public void accelerateNegative() {
        accelerate(-DEFAULT_ACCELERATION);
    }

    public void decelerate(double deceleration) {
        if(getPower() > 0)
            accelerate(-deceleration);
        else if(getPower() < 0)
            accelerate(deceleration);
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
            accelerateTo(power);
        else
            accelerateTo(-power);
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
        updateTelemetry();
    }
    public void setUpPosition() {
        upPosition = getPosition();
        updateTelemetry();
    }
    public double getDownPosition() { return downPosition; }
    public double getUpPosition() { return upPosition; }

    // hold at the current position
    public void hold() {
        if(!holding) {
            holdPosition = getPosition();
            holding = true;
            decelerate();
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
    @SuppressLint("DefaultLocale")
    public void updateTelemetry() {
        if(telemetry != null)
            telemetry.setValue("%s %s [%s]", getState(), isUsingEncoder() ? String.format("(%d)", getPosition()) : null, getPowerAsString());
    }
}
