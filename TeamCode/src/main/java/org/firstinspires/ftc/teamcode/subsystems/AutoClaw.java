package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

// claw that releases a pixel in auto
public class AutoClaw extends CustomServo {
    public static final String IN = "IN", OUT = "OUT", PARTLY_OUT = "PARTLY OUT";
    private static final double INCREMENT = .1;
    private static final double IN_PSN = 0, OUT_PSN = .56;
    private final ScheduledExecutorService executorService;
    // constructor
    public AutoClaw(HardwareMap hm, String name) {
        super(hm, name, Math.min(IN_PSN, OUT_PSN), Math.max(IN_PSN, OUT_PSN)); // calls parent constructor
        executorService = Executors.newSingleThreadScheduledExecutor();
    }
    public void outIncrementally() { changePosition(INCREMENT); }
    public void inIncrementally() { changePosition(-INCREMENT); }
    public void out() {
        setPosition(OUT_PSN);
    }
    public void in() {
        setPosition(IN_PSN);
    }
    public void outAndIn() {
        out();
        executorService.schedule(this::in, 500, TimeUnit.MILLISECONDS);
    }

    // returns the current status of the gripper as a String
    public String getState() {
        double psn = getPosition();
        if(Math.abs(psn - IN_PSN) < .02)
            return IN;
        else if(Math.abs(psn - OUT_PSN) < .02)
            return OUT;
        else
            return PARTLY_OUT;
    }
}
