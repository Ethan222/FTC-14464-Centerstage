package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

// controls the servo that rotates the outtake mechanism
public class Rotator extends CustomServo {
    private static final double INCREMENT = .01;    // this relates to how fast the gripper will move
    private double EXTENDED_PSN, RETRACTED_PSN;
    // constructor
    public Rotator(HardwareMap hm, String name, double retractedPsn, double extendedPsn) {
        super(hm, name, 0, 1); // calls parent constructor
        RETRACTED_PSN = retractedPsn;
        EXTENDED_PSN = extendedPsn;
    }
    public Rotator(HardwareMap hm, String name) {
        this(hm, name, 0, 1);
    }
    public void rotateFully() {
        setPosition(EXTENDED_PSN);
    }
    public void retractFully() {
        setPosition(RETRACTED_PSN);
    }
    public void rotateIncrementally() { changePosition(-INCREMENT); }
    public void retractIncrementally() { changePosition(INCREMENT); }

    // returns the current status as a String
    public String getStatus() {
        double psn = getPosition();
        if(Math.abs(psn - RETRACTED_PSN) < .03)
            return "retracted";
        else if(Math.abs(psn - EXTENDED_PSN) < .03)
            return "extended";
        else if(psn > RETRACTED_PSN)
            return "extra retracted";
        else if(psn < EXTENDED_PSN)
            return "extra extended";
        else
            return "partly extended";
    }
}
