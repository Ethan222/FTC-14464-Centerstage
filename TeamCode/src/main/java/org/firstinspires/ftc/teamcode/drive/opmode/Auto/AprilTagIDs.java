package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

public class AprilTagIDs {
    //public final static int BlueBackdropLeft = 1;

    public Backdrop blueBackdrop, redBackdrop;
    public AprilTagIDs() {
        blueBackdrop = new Backdrop("blue", 1, 2, 3);
        redBackdrop = new Backdrop("red", 4, 5, 6);
    }
}

class Backdrop {
    public String color;
    public int left, center, right;
    public int[] arr;
    public Backdrop(String clr, int l, int c, int r) {
        color = clr;
        left = l;
        center = c;
        right = r;
        arr = new int[]{l, c, r};
    }
}