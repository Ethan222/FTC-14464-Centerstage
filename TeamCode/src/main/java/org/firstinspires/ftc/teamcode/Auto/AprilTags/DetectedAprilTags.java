package org.firstinspires.ftc.teamcode.Auto.AprilTags;

import org.openftc.apriltag.AprilTagDetection;

public class DetectedAprilTags {
    public AprilTagDetection left, center, right;
    public AprilTagDetection[] tags = {left, center, right};
    public void setLeft(AprilTagDetection l) {
        left = l;
    }

    public void setCenter(AprilTagDetection c) {
        center = c;
    }

    public void setRight(AprilTagDetection r) {
        right = r;
    }
}
