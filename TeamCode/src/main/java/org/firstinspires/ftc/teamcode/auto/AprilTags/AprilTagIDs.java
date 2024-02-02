package org.firstinspires.ftc.teamcode.auto.AprilTags;

import org.firstinspires.ftc.teamcode.auto.enums.Alliance;

public class AprilTagIDs {
    public static Backdrop blueBackdrop = new Backdrop(Alliance.BLUE, 1, 2, 3);
    public static Backdrop redBackdrop = new Backdrop(Alliance.RED, 4, 5, 6);
    public static Backdrop getBackdrop(Alliance alliance) {
        if(alliance == Alliance.BLUE)
            return blueBackdrop;
        else
            return redBackdrop;
    }
}