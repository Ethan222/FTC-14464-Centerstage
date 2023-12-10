package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Rotator {
    private CRServo rotator;
    private static final double DEFAULT_SPEED = 1;
    public Rotator(HardwareMap hm, String name) {
        rotator = hm.get(CRServo.class, name);
    }
    public void setPower(double power) {
        rotator.setPower(power);
    }
    public void rotate(double power) {
        setPower(power);
    }
    public void rotate() {
        rotate(DEFAULT_SPEED);
    }
    public void unrotate(double power) {
        setPower(-power);
    }
    public void unrotate() {
        unrotate(DEFAULT_SPEED);
    }
    public void stop() {
        rotator.setPower(0);
    }
    public double getPower() {
        return rotator.getPower();
    }
}
