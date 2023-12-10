package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

// controls the motor that turns/flips the arm
public class ArmFlipper extends Motor {
    public ArmFlipper(HardwareMap hardwareMap, String name)
    {
        super(hardwareMap, name);
    }
    public void flip(double power) {
        setPower(power);
    }
    public void flip() {
        flip(DEFAULT_SPEED);
    }
    public void unflip(double power) {
        setPower(-power);
    }
    public void unflip() {
        unflip(DEFAULT_SPEED);
    }
}
