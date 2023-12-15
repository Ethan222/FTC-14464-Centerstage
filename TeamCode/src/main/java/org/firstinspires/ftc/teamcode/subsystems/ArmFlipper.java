package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

// controls the motor that turns/flips the arm
// TODO: add encoder so arm can't go down after it's already all the way down
public class ArmFlipper extends Motor { // inherits from the Motor parent class
    public ArmFlipper(HardwareMap hardwareMap, String name) // constructor
    {
        super(hardwareMap, name);       // calls parent constructor
    }
    public void flip(double power) {    // a negative power flips the arm
        setPower(-power);               // the setPower() method is inherited from Motor
    }
    public void flip() {     // function overload: if no speed given, use default speed
        flip(DEFAULT_SPEED); // DEFAULT_SPEED is inherited from Motor
    }
    public void unflip(double power) {  // a positive power unflips the arm
        setPower(power);
    }
    public void unflip() {              // function overload
        unflip(DEFAULT_SPEED);
    }
}
