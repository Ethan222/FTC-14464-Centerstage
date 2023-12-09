package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    public CustomServo gripper, rotator;
    private static final double GRIPPER_INCREMENT = .01, ROTATOR_INCREMENT = .01;
    public Claw(HardwareMap hm, String gripperName, String rotatorName) {
        gripper = new CustomServo(hm, gripperName);
        rotator = new CustomServo(hm, rotatorName, 0, .5);
    }
    public void gripIncrementally() {
        gripper.changePosition(-GRIPPER_INCREMENT);
    }
    public void ungripIncrementally() {
        gripper.changePosition(GRIPPER_INCREMENT);
    }
    public void gripFully() {
        gripper.goToLeft();
    }
    public void ungripFully() {
        gripper.goToRight();
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
}
