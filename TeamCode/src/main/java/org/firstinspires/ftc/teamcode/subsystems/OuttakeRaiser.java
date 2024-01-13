package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

// controls the motor that raises the outtake
// TODO: add encoder so arm can't go down after it's already all the way down
public class OuttakeRaiser extends Motor { // inherits from the Motor parent class
    public OuttakeRaiser(HardwareMap hardwareMap, String name) // constructor
    {
        super(hardwareMap, name);       // calls parent constructor
    }
    public void up(double power) {    // a negative power flips the arm
        setPower(power);               // the setPower() method is inherited from Motor
    }
    public void up() {     // function overload: if no speed given, use default speed
        up(DEFAULT_SPEED); // DEFAULT_SPEED is inherited from Motor
    }
    public void down(double power) {  // a positive power unflips the arm
        setPower(-power);
    }
    public void down() {              // function overload
        down(DEFAULT_SPEED);
    }
}
