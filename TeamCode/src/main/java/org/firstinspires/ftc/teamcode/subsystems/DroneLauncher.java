package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

public class DroneLauncher extends CustomServo {
    public DroneLauncher(HardwareMap hardwareMap, String name) {
        super(hardwareMap, name, .6, 1);
    }
    public void launch() {
        decreasePosition();
    }
    public void reset() {
        increasePosition();
    }
    public String getStatus() {
        String status = super.getStatus();
        if(status.equals(Status.LEFT.toString()))
            return "forward";
        else if(status.equals(Status.RIGHT.toString()))
            return "back (ready)";
        else
            return "partly forward";
    }
}
