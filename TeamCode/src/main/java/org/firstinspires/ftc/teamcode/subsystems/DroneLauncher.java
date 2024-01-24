package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

public class DroneLauncher extends CustomServo {
    public final static String BACK = "BACK", PARTLY_FORWARD = "PARTLY FORWARD", FORWARD = "FORWARD";
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
        if(status.equals(LEFT))
            return FORWARD;
        else if(status.equals(RIGHT))
            return BACK;
        else
            return PARTLY_FORWARD;
    }
}
