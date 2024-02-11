package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

// controls the motor that raises the outtake
public class Outtake extends Motor { // inherits from the Motor parent class
    public OuttakeRotator rotator;
    public static int[] POSITIONS = {490, 750, 2000};
    public Outtake(HardwareMap hardwareMap, String motorName, String servoName) // constructor
    {
        super(hardwareMap, motorName, true, 0, POSITIONS[0]);       // calls parent constructor
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator = new OuttakeRotator(hardwareMap, servoName, .4, .72);
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

    public void setPosition(int i) {
        POSITIONS[i] = getPosition();
    }

    public String getTelemetry() {
        return String.format(Locale.ENGLISH, "%s %s (%d) [%s]", getState().toString(), isHolding() ? "(holding at " + getHoldPosition() + ")" : "", getPosition(), getPowerAsString());
    }
}
