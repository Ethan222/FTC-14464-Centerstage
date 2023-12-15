package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

// represents the servo that controls the gripper
public class Gripper extends CustomServo {
    private static final double INCREMENT = .01;    // this relates to how fast the gripper will move
    private static final double OPEN_PSN = .23, CLOSED_PSN = .45;
    public Gripper(HardwareMap hm, String gripperName) {    // constructor
        super(hm, gripperName, OPEN_PSN, CLOSED_PSN);       // calls parent constructor
    }
    public void gripIncrementally() { changePosition(INCREMENT); }       // grips a little at a time
    public void ungripIncrementally() { changePosition(-INCREMENT); }    // ungrips a little at a time
    public void gripFully() {
        goToRight();
    }       // completely closes the gripper
    public void ungripFully() {
        goToLeft();
    }      // completely opens the gripper
    public String getStatus() {         // returns the current status of the gripper as a String
        double psn = getPosition();     // returns either "fully open", "fully closed", or "partially open"
        if (Math.abs(psn - OPEN_PSN) < .03) {
            return "open";
        } else if (Math.abs(psn - CLOSED_PSN) < .03) {
            return "closed";
        } else {
            return "partially open";
        }
    }
}
