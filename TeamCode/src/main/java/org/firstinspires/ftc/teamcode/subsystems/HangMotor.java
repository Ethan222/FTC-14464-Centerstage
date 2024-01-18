package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// controls motor that hangs
public class HangMotor extends Motor {     // inherits from Motor
    public HangMotor(HardwareMap hm, String name) {             // constructor
        super(hm, name, true, 0, 11000);      // calls parent constructor with default speed and max speed
    }
    public void up(double power) {
        setPower(-power);            // the setPower() method is inherited from Motor
    }
    public void down(double power) { setPower(power); }
    public void up() {
        up(1);
    }
    public void down() {
        down(1);
    }
}
