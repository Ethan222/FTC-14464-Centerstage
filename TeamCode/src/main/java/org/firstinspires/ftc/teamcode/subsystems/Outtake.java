package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// controls the motor that raises the outtake
public class Outtake extends Motor { // inherits from the Motor parent class
    public Rotator rotator;
    public Outtake(HardwareMap hardwareMap, String motorName, String servoName) // constructor
    {
        super(hardwareMap, motorName, true, 10, 500);       // calls parent constructor
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator = new Rotator(hardwareMap, servoName, .52, .11);
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
    public void accelerateUp(double acceleration) {
        acceleratePositive(acceleration);
    }
    public void accelerateUp() {
        accelerateUp(DEFAULT_ACCELERATION);
    }
    public void accelerateDown(double acceleration) {
        accelerateNegative(acceleration);
    }
    public void accelerateDown() {
        accelerateDown(DEFAULT_ACCELERATION);
    }
}
