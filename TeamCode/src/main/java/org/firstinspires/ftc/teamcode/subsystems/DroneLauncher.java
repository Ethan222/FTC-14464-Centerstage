package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.CustomServo.LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.CustomServo.RIGHT;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class DroneLauncher {
    public final static String BACK = "BACK", PARTLY_FORWARD = "PARTLY FORWARD", FORWARD = "FORWARD";
    public final static String DOWN = "DOWN", PARTLY_UP = "PARTLY UP", UP = "UP";
    public final CustomServo launcher, rotator;
    public DroneLauncher(HardwareMap hardwareMap, String launcherName, String rotatorName) {
        launcher = new CustomServo(hardwareMap, launcherName, 0, .45);
        rotator = new CustomServo(hardwareMap, rotatorName, .72, .86, .005);
    }
    public void launch() {
        launcher.increasePosition();
    }
    public void reset() {
        launcher.decreasePosition();
    }
    public void rotateUp() { rotator.decreasePosition(); }
    public void rotateDown() { rotator.increasePosition(); }
    public double getPosition() { return launcher.getPosition(); }
    public String getState() {
        String state = launcher.getState();
        if(state.equals(LEFT))
            return BACK;
        else if(state.equals(RIGHT))
            return FORWARD;
        else
            return PARTLY_FORWARD;
    }
    public double getRotation() {
        return rotator.getPosition();
    }
    public double getRotationPercent() {
        return 1 - rotator.getPercent();
    }
    public String getRotationState() {
        double rotation = getRotation();
        if(Math.abs(rotation - rotator.getRightPosition()) < .02)
            return DOWN;
        else if(Math.abs(rotation - rotator.getLeftPosition()) < .02)
            return UP;
        else
            return PARTLY_UP;
    }
}
