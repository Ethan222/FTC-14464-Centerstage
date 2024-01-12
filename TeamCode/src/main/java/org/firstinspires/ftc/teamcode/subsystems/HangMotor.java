package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

// controls motor that hangs
public class HangMotor extends Motor {     // inherits from Motor
    public HangMotor(HardwareMap hm, String name) {             // constructor
        super(hm, name);      // calls parent constructor with default speed and max speed
    }
    // up() raises the arm and down() lowers it. Each method has 2 overloads based on whether or not it is passed an argument.
    public void up(double power) {
        setPower(-power);            // the setPower() method is inherited from Motor
    }
    public void up() {
        setPower(DEFAULT_SPEED);    // DEFAULT_SPEED, which was set to .5 above in the constructor, is inherited from Motor
    }
    public void down(double power) { setPower(power); }    // a negative power lowers the arm
    public void down() { down(DEFAULT_SPEED); }
}
