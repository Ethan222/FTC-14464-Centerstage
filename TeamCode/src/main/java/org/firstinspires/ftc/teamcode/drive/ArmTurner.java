package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

// controls the motor that turns/pivots/flips the arm
public class ArmTurner extends CustomMotor {
    public ArmTurner(HardwareMap hardwareMap, String name)
    {
        super(hardwareMap, name);
    }
    public void turn(double power) {
        setPower(-power);
    }
    public void turn() {
        turn(DEFAULT_SPEED);
    }
    public void unturn(double power) {
        setPower(power);
    }
    public void unturn() {
        unturn(DEFAULT_SPEED);
    }
}
