package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

public class DroneLauncher extends CustomServo {
    public final static String BACK = "BACK", PARTLY_FORWARD = "PARTLY FORWARD", FORWARD = "FORWARD";
    public DroneLauncher(HardwareMap hardwareMap, String name) {
        super(hardwareMap, name, 0, .45);
    }
    public void launch() {
        increasePosition();
    }
    public void reset() {
        decreasePosition();
    }
    public String getStatus() {
        String status = super.getStatus();
        if(status.equals(LEFT))
            return BACK;
        else if(status.equals(RIGHT))
            return FORWARD;
        else
            return PARTLY_FORWARD;
    }
}
