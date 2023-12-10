package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gripper {
    public CustomServo gripper;
    private static final double GRIPPER_INCREMENT = .01;
    public Gripper(HardwareMap hm, String gripperName) {
        gripper = new CustomServo(hm, gripperName, 0, .45);
    }
    public void gripIncrementally() {
        gripper.changePosition(GRIPPER_INCREMENT);
    }
    public void ungripIncrementally() {
        gripper.changePosition(-GRIPPER_INCREMENT);
    }
    public void gripFully() {
        gripper.goToRight();
    }
    public void ungripFully() {
        gripper.goToLeft();
    }

    public String gripperStatus() {
        double psn = gripper.getPosition();
        if (psn == 0) {
            return "fully open";
        } else if (psn == .45) {
            return "fully closed";
        } else {
            return "partially open";
        }
    }
}
