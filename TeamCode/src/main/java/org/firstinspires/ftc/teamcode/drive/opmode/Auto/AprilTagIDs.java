package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

public class AprilTagIDs {
    //public final static int BlueBackdropLeft = 1;

    public Backdrop blueBackdrop, redBackdrop;
    public AprilTagIDs() {
        blueBackdrop = new Backdrop(1, 2, 3);
        redBackdrop = new Backdrop(4, 5, 6);
    }
}

class Backdrop {
    public int left, center, right;
    public Backdrop(int l, int c, int r) {
        left = l;
        center = c;
        right = r;
    }
}