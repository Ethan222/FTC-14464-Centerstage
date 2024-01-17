package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

// represents a servo that controls a gripper
public class Claw extends CustomServo {
    private static final double INCREMENT = .01;    // this relates to how fast the gripper will move
    private double UP_PSN, DOWN_PSN;
    // constructor
    public Claw(HardwareMap hm, String name, double upPsn, double downPsn) {
        super(hm, name, Math.min(upPsn, downPsn), Math.max(upPsn, downPsn)); // calls parent constructor
        UP_PSN = upPsn;
        DOWN_PSN = downPsn;
    }
    public Claw(HardwareMap hm, String name) {
        this(hm, name, 0, 1);
    }
    public void down() {
        setPosition(DOWN_PSN);
    }       // completely closes the claw
    public void up() {
        setPosition(UP_PSN);
    }      // completely opens the claw
    public void downIncrementally() { changePosition(INCREMENT); }
    public void upIncrementally() { changePosition(-INCREMENT); }

    // returns the current status of the claw as a String
    public String getStatus() {
        String status = super.getStatus();
        if(status.equals(Status.LEFT.toString()))
            return UP_PSN < DOWN_PSN ? "up" : "down";
        else if(status.equals(Status.RIGHT.toString()))
            return UP_PSN > DOWN_PSN ? "up" : "down";
        else
            return "partway up";
    }
}
