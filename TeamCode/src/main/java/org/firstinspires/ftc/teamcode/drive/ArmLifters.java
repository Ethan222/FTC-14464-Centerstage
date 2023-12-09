package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

// controls the 2 motors that raise the arm
public class ArmLifters extends Motor {
    public ArmLifters(HardwareMap hm, String name1, String name2) {
        super(hm, new String[]{name1, name2});
    }
    public void up(double power) {
        setPower(power);
    }
    public void up() {
        setPower(DEFAULT_SPEED);
    }
    public void down(double power) {
        setPower(-power);
    }
    public void down() {
        down(DEFAULT_SPEED);
    }
}
