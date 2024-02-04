package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

// class to control the intake mechanism
public class Intake extends Motor { // inherits from parent class Motor
    public CustomServo lowerer;     // servo to raise/lower the intake
    private static final double UP_PSN = .5, DOWN_PSN = 1;   // up and down positions of the servo
    public Intake(HardwareMap hardwareMap, String motorName, String servoName) {        // constructor
        super(hardwareMap, motorName);   // calls parent constructor to initialize intake motor
        // initialize lowerer servo
        lowerer = new CustomServo(hardwareMap, servoName, Math.min(DOWN_PSN, UP_PSN), Math.max(DOWN_PSN, UP_PSN));
    }
    public void in(double power) {
        setPower(power);
    } // positive intakes
    public void in() {
        in(1);
    }
    public void out(double power) { setPower(-power); } // negative outtakes
    public void out() { out(1); }
    public void lower() { // lower intake
        lowerer.setPosition(DOWN_PSN);
    }
    public void raise() { // raise intake
        lowerer.setPosition(UP_PSN);
    }
    public void lower(double val) { lowerer.changePosition(val); }
    public void raise(double val) { lowerer.changePosition(-val); }

    // return the status of the intake (up or down)
    public State getState() {
        double psn = lowerer.getPosition();
        if(Math.abs(psn - UP_PSN) < .03)
            return State.UP;
        else if(Math.abs(psn - DOWN_PSN) < .03)
            return State.DOWN;
        else
            return State.UNSURE;
    }

    public String getTelemetry() {
        return String.format(Locale.ENGLISH, "%s (%.2f) [%s]", getState().toString(), lowerer.getPosition(), getPowerAsString());
    }
}
