package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// controls the 2 motors that raise the arm
public class ArmRaiser extends Motor {     // inherits from Motor
    public ArmRaiser(HardwareMap hm, String name) {             // constructor
        super(hm, name, .5, 1);      // calls parent constructor with default speed and max speed
    }
    // up() raises the arm and down() lowers it. Each method has 2 overloads based on whether or not it is passed an argument.
    public void up(double power) {
        setPower(power);            // the setPower() method is inherited from Motor
    }
    public void up() {
        setPower(DEFAULT_SPEED);    // DEFAULT_SPEED, which was set to .5 above in the constructor, is inherited from Motor
    }
    public void down(double power) { setPower(-power); }    // a negative power lowers the arm
    public void down() { down(DEFAULT_SPEED); }
}
