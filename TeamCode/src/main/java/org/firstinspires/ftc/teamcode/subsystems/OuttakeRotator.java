package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

// controls the servo that rotates the outtake mechanism
public class OuttakeRotator extends CustomServo {
    public final static String EXTRA_RETRACTED = "EXTRA RETRACTED", RETRACTED = "RETRACTED", PARTLY_EXTENDED = "PARTLY EXTENDED",
            EXTENDED = "EXTENDED", EXTRA_EXTENDED = "EXTRA EXTENDED";
    private static final double INCREMENT = .005;
    public double EXTENDED_PSN, RETRACTED_PSN;
    // constructor
    public OuttakeRotator(HardwareMap hm, String name, double retractedPsn, double extendedPsn) {
        super(hm, name, 0, 1); // calls parent constructor
        RETRACTED_PSN = retractedPsn;
        EXTENDED_PSN = extendedPsn;
    }
    public OuttakeRotator(HardwareMap hm, String name) {
        this(hm, name, 0, 1);
    }
    public void rotateFully() {
        setPosition(EXTENDED_PSN);
    }
    public void retractFully() {
        setPosition(RETRACTED_PSN);
    }
    public void rotateIncrementally() { changePosition(INCREMENT); }
    public void retractIncrementally() { changePosition(-INCREMENT); }

    // returns the current status as a String
    public String getStatus() {
        double psn = getPosition();
        if(Math.abs(psn - RETRACTED_PSN) < .03)
            return RETRACTED;
        else if(Math.abs(psn - EXTENDED_PSN) < .03)
            return EXTENDED;
        else if(psn < RETRACTED_PSN)
            return EXTRA_RETRACTED;
        else if(psn > EXTENDED_PSN)
            return EXTRA_EXTENDED;
        else
            return PARTLY_EXTENDED;
    }
}
