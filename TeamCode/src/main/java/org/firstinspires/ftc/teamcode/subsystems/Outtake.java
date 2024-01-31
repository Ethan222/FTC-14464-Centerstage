package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// controls the motor that raises the outtake
public class Outtake extends Motor { // inherits from the Motor parent class
    public OuttakeRotator rotator;
    public static final int UP_POSITION_1 = 380, UP_POSITION_2 = 600;
    public Outtake(HardwareMap hardwareMap, String motorName, String servoName) // constructor
    {
        super(hardwareMap, motorName, true, 0, UP_POSITION_1);       // calls parent constructor
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator = new OuttakeRotator(hardwareMap, servoName, .49, .83);
    }
    public void up(double power) {
        setPower(power);
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
