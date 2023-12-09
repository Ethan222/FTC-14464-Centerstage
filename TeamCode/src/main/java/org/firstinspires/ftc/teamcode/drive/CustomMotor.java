package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Generic motor class that can handle one or more motors
public abstract class CustomMotor {
    protected final DcMotorEx[] motors;
    protected final double DEFAULT_SPEED, MAX_SPEED;
    public CustomMotor(HardwareMap hm, String[] names, double defaultSpeed, double maxSpeed) {
        motors = new DcMotorEx[names.length];
        for(int i = 0; i < motors.length; i++) {
            motors[i] = hm.get(DcMotorEx.class, names[i]);
        }
        DEFAULT_SPEED = defaultSpeed;
        MAX_SPEED = maxSpeed;
    }
    public CustomMotor(HardwareMap hm, String name, double defaultSpeed, double maxSpeed) {
        this(hm, new String[]{name}, defaultSpeed, maxSpeed);
    }
    public CustomMotor(HardwareMap hm, String[] names) {
        this(hm, names, 1, 1);
    }
    public CustomMotor(HardwareMap hm, String name) {
        this(hm, new String[]{name});
    }
    public void setPower(double power) {
        for(DcMotorEx motor : motors) {
            motor.setPower(power * MAX_SPEED);
        }
    }
    public void stop() { setPower(0); }
    @SuppressLint("DefaultLocale")
    public String getPower() {
        if(motors.length == 1) {
            return String.format("%.2f", motors[0].getPower());
        } else {
            StringBuilder powers = new StringBuilder();
            for (DcMotorEx motor : motors) {
                powers.append(String.format("%.3f, ", motor.getPower()));
            }
            String ret = powers.toString();
            return ret.substring(0, ret.length() - 2);
        }
    }
}
