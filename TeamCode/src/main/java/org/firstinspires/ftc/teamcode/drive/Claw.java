package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    public CustomServo gripper, rotator;
    private static final double GRIPPER_INCREMENT = .01, ROTATOR_INCREMENT = .01;
    public Claw(HardwareMap hm, String gripperName, String rotatorName) {
        gripper = new CustomServo(hm, gripperName, 0, .45);
        rotator = new CustomServo(hm, rotatorName, 0, .35);
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
    public void rotateIncrementally(double increment) {
        rotator.changePosition(increment * ROTATOR_INCREMENT);
    }
    public void rotateIncrementally() {
        rotator.changePosition(ROTATOR_INCREMENT);
    }
    public void unrotateIncrementally(double increment) {
        rotator.changePosition(-increment * ROTATOR_INCREMENT);
    }
    public void unrotateIncrementally() {
        rotator.changePosition(-ROTATOR_INCREMENT);
    }
    public void rotateFully() {
        rotator.goToRight();
    }
    public void unrotateFully() {
        rotator.goToLeft();
    }

    public String gripperStatus() {
        double psn = gripper.getPosition();
        if(psn == 0) {
            return "fully open";
        }
        else if(psn == .45) {
            return "fully closed";
        }
        else {
            return "partially open";
        }
    }
    public String rotatorStatus() {
        double psn = rotator.getPosition();
        if(psn > .3) {
            return "fully extended";
        } else if(psn == 0) {
            return "down";
        }
        else {
            return "partially extended";
        }
    }
}
