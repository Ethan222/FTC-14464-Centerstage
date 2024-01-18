package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Generic motor class that can handle one motor
public abstract class Motor {
    protected final DcMotorEx motor;
    private boolean usingEncoder;
    private int downPosition, upPosition;
    public Motor(HardwareMap hm, String name, boolean usingEncoder, int downPsn, int upPsn) { // constructor
        motor = hm.get(DcMotorEx.class, name); // get motor from the hardwareMap
        downPosition = downPsn;
        upPosition = upPsn;
        if(usingEncoder)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public Motor(HardwareMap hm, String name) {
        this(hm, name, false, 0, 0);
    }
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
    public void setPower(double power) {
            motor.setPower(power);
    }
    public void stop() { setPower(0); }
    @SuppressLint("DefaultLocale")
    public String getPower() {
        return String.format("%.1f", motor.getPower());
    }
    public int getPosition() {
        return motor.getCurrentPosition();
    }
    public boolean isBusy() {
        return motor.isBusy();
    }

    public void goToPosition(int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(position > getPosition())
            setPower(1);
        else
            setPower(-1);
    }
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
}
