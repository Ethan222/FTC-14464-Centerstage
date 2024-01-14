package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Generic motor class that can handle one motor
public abstract class Motor {
    public final DcMotorEx motor;
    protected final double DEFAULT_SPEED; // the default speed when one isn't specified
    protected final double MAX_SPEED; // max speed motor can run at without causing damage
    public Motor(HardwareMap hm, String name, double defaultSpeed, double maxSpeed) { // constructor
        motor = hm.get(DcMotorEx.class, name); // get motor from the hardwareMap
        DEFAULT_SPEED = defaultSpeed;
        MAX_SPEED = maxSpeed;
    }
    public Motor(HardwareMap hm, String name) {
        this(hm, name, 1, 1);
    }
    public void setPower(double power) {
            motor.setPower(power * MAX_SPEED);
    }
    public void stop() { setPower(0); }
    @SuppressLint("DefaultLocale")
    public String getPower() {
        return String.format("%.1f", motor.getPower());
    }
}
