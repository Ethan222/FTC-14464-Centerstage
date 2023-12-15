package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class DroneLauncher extends CustomServo {
    private static final double INCREMENT = .01;
    public DroneLauncher(HardwareMap hardwareMap, String name) {
        super(hardwareMap, name, 0, .5);
    }
    public void launch() {
        changePosition(INCREMENT);
    }
    public void reset() {
        changePosition(-INCREMENT);
    }
}
