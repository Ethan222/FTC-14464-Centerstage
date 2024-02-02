package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.CustomServo.LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.CustomServo.RIGHT;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncher {
    public final static String BACK = "BACK", PARTLY_FORWARD = "PARTLY FORWARD", FORWARD = "FORWARD";
    public final static String DOWN = "DOWN", PARTLY_UP = "PARTLY UP", UP = "UP";
    public final CustomServo launcher, rotator;
    private Telemetry.Item launcherTelemetry, rotatorTelemetry;
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
    public double getLauncherPosition() { return launcher.getPosition(); }
    public String getLauncherState() {
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

    public void setLauncherTelemetry(Telemetry.Item item) {
        launcherTelemetry = item;
    }
    public void setRotatorTelemetry(Telemetry.Item item) {
        rotatorTelemetry = item;
    }
    public void updateLauncherTelemetry() {
        launcherTelemetry.setValue("%s [%.2f] (%.2f)", getLauncherState(), launcher.getPercent(), launcher.getPosition());
    }
    public void updateRotatorTelemetry() {
        rotatorTelemetry.setValue("%s [%.2f] (%.2f)", getRotationState(), getRotationPercent(), getRotation());
    }
}
