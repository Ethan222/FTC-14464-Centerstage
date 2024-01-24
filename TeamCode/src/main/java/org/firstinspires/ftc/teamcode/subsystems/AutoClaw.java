package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

// claw that releases a pixel in auto
public class AutoClaw extends CustomServo {
    public final static String IN = "IN", OUT = "OUT", PARTLY_OUT = "PARTLY OUT";
    private static final double INCREMENT = .01;    // how fast the claw will move
    private double IN_PSN, OUT_PSN;
    // constructor
    public AutoClaw(HardwareMap hm, String name, double inPsn, double outPsn) {
        super(hm, name, inPsn, outPsn); // calls parent constructor
        IN_PSN = inPsn;
        OUT_PSN = outPsn;
    }
    public AutoClaw(HardwareMap hm, String name) {
        this(hm, name, 0, 1);
    }
    public void in() {
        goToLeft();
    }       // completely closes the gripper
    public void out() {
        goToRight();
    }      // completely opens the gripper
    public void inIncrementally() { changePosition(-INCREMENT); }       // grips a little at a time
    public void outIncrementally() { changePosition(INCREMENT); }

    // returns the current status of the gripper as a String
    public String getStatus() {
        String status = super.getStatus();
        if(status.equals(CustomServo.LEFT))
            return IN;
        else if(status.equals(CustomServo.RIGHT))
            return OUT;
        else
            return PARTLY_OUT;
    }
}
