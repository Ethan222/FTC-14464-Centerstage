package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// controls the motor that raises the outtake
public class OuttakeRaiser extends Motor { // inherits from the Motor parent class
    public OuttakeRaiser(HardwareMap hardwareMap, String name) // constructor
    {
        super(hardwareMap, name, true, 10, 540);       // calls parent constructor
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void up(double power) {    // a negative power flips the arm
        setPower(power);               // the setPower() method is inherited from Motor
    }
    public void down(double power) {  // a positive power unflips the arm
        setPower(-power);
    }
    public void up() {
        up(1);
    }
    public void down() {
        down(1);
    }
}
