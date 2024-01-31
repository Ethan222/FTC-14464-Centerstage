package org.firstinspires.ftc.teamcode.Auto.AprilTags;

import org.firstinspires.ftc.teamcode.Auto.Alliance;
import org.firstinspires.ftc.teamcode.Auto.Location;

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

class Backdrop {
    public Alliance alliance;
    public int LEFT, CENTER, RIGHT;
    public Backdrop(Alliance alliance, int l, int c, int r) {
        this.alliance = alliance;
        LEFT = l; CENTER = c; RIGHT = r;
    }
    public int getId(Location location) {
        switch(location) {
            case LEFT:
                return LEFT;
            case CENTER:
                return CENTER;
            case RIGHT:
                return RIGHT;
        }
        return 0;
    }
}