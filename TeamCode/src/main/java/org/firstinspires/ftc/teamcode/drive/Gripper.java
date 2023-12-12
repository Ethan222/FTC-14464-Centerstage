package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

// represents the servo that controls the gripper
public class Gripper {
    public CustomServo gripper;                             // the gripper is an instance of the CustomServo class
    private static final double GRIPPER_INCREMENT = .01;    // this relates to how fast the gripper will moe
    public Gripper(HardwareMap hm, String gripperName) {    // constructor
        gripper = new CustomServo(hm, gripperName, 0, .45);     // instantiates gripper object
    }
    public void gripIncrementally() { gripper.changePosition(GRIPPER_INCREMENT); }       // grips a little at a time
    public void ungripIncrementally() { gripper.changePosition(-GRIPPER_INCREMENT); }    // ungrips a little at a time
    public void gripFully() {
        gripper.goToRight();
    }       // completely closes the gripper
    public void ungripFully() {
        gripper.goToLeft();
    }      // completely opens the gripper
    public String gripperStatus() {             // returns the current status of the gripper as a String
        double psn = gripper.getPosition();     // returns either "fully open", "fully closed", or "partially open"
        if (psn == 0) {
            return "fully open";
        } else if (psn == .45) {
            return "fully closed";
        } else {
            return "partially open";
        }
    }
}
