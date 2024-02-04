package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

// controls the motor that raises the outtake
public class Outtake extends Motor { // inherits from the Motor parent class
    public OuttakeRotator rotator;
    public static int POSITION_1 = 490, POSITION_2 = 550+200;
    public Outtake(HardwareMap hardwareMap, String motorName, String servoName) // constructor
    {
        super(hardwareMap, motorName, true, 0, POSITION_1);       // calls parent constructor
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator = new OuttakeRotator(hardwareMap, servoName, .26, .58);
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
        accelerate(acceleration);
    }
    public void accelerateUp() {
        accelerateUp(DEFAULT_ACCELERATION);
    }
    public void accelerateDown(double acceleration) {
        accelerate(-acceleration);
    }
    public void accelerateDown() {
        accelerateDown(DEFAULT_ACCELERATION);
    }

    public void setPosition2() {
        POSITION_2 = getPosition();
    }

    public String getTelemetry() {
        return String.format(Locale.ENGLISH, "%s %s (%d) [%s]", getState().toString(), isHolding() ? "(holding at " + getHoldPosition() + ")" : "", getPosition(), getPowerAsString());
    }
}
