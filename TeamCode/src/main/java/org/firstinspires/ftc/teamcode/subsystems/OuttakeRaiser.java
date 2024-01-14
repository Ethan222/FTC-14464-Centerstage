package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// controls the motor that raises the outtake
public class OuttakeRaiser extends Motor { // inherits from the Motor parent class
    private static final int POSITION_1 = 410;
    public OuttakeRaiser(HardwareMap hardwareMap, String name) // constructor
    {
        super(hardwareMap, name);       // calls parent constructor
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
    public int getPosition() {
        return motor.getCurrentPosition();
    }
    public void goToPosition(LinearOpMode opMode, int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(position > getPosition())
            up();
        else
            down();
    }
    public void goToPosition1(LinearOpMode opMode) {
        goToPosition(opMode, POSITION_1);
    }
    public void goDown(LinearOpMode opMode) {
        goToPosition(opMode, 10);
    }
    public boolean isBusy() {
        return motor.isBusy();
    }
}
