package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// controls the 2 motors that raise the arm
public class ArmRaisers extends Motor {
    public ArmRaisers(HardwareMap hm, String name1, String name2) {
        super(hm, new String[]{name1, name2}, .7, 1);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE); // reverse left arm lifter
        for(DcMotorEx motor: motors)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void up(double power) {
        setPower(power);
    }
    public void up() {
        setPower(DEFAULT_SPEED);
    }
    public void down(double power) {
        setPower(-power);
    }
    public void down() {
        down(DEFAULT_SPEED);
    }
}
