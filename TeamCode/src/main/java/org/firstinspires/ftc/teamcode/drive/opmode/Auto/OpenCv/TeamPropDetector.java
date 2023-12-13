package org.firstinspires.ftc.teamcode.drive.opmode.Auto.OpenCv;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.List;

public class TeamPropDetector extends OpenCvPipeline {
    private final OpenCvCamera camera;
    private final int camW = 800, camH = 448;
    private Location location = Location.LEFT;
    private List<Integer> PROP_COLOR = Arrays.asList(255, 0, 0); // rgb
    private int toggleOverlay = 1;
    private Mat original;
    private Mat zone1, zone2, zone3;
    private Scalar avgColor1, avgColor2, avgColor3;
    private double min_difference = 0;

    public TeamPropDetector(Telemetry telemetry, HardwareMap hardwareMap) {
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        camera.setPipeline(this);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(camW, camH, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });
    }
    // will be called once at the start
    @Override
    public void init(Mat input) {
        // create duplicate of original frame with no edits
        original = input.clone();

        // define zones
        Rect one = new Rect(5, 600, 195, 140);
        Rect two = new Rect(150, 600, 195, 140);
        Rect three = new Rect(300, 600, 195, 140);
        Imgproc.rectangle(input, one, new Scalar(0, 0, 255), 2);
        Imgproc.rectangle(input, two, new Scalar(0, 0, 255), 2);
        Imgproc.rectangle(input, three, new Scalar(0, 0, 255), 2);
        // Rect(top left x, top left y, w, h)
        zone1 = input.submat(one);
        zone2 = input.submat(two);
        zone3 = input.submat(three);
    }
    @Override
    public Mat processFrame(Mat input) {

        // average the colors in the zones
        avgColor1 = Core.mean(zone1);
        avgColor2 = Core.mean(zone2);
        avgColor3 = Core.mean(zone3);

        // put averaged colors on zones so we can see on the camera stream
        zone1.setTo(avgColor1);
        zone2.setTo(avgColor2);
        zone3.setTo(avgColor3);

        // find the distance of each color from the target color
        double distance1 = color_difference(avgColor1, PROP_COLOR);
        double distance2 = color_difference(avgColor2, PROP_COLOR);
        double distance3 = color_difference(avgColor3, PROP_COLOR);

        // find whichever zone is the closest to the target color
        min_difference = Math.min(distance3, Math.min(distance1, distance2));

        if (min_difference == distance1) {
            location = Location.LEFT;

        } else if (min_difference == distance2) {
            location = Location.CENTER;
        } else {
            location = Location.RIGHT;
        }
        
        // allow for the showing of the average colors on the stream
        if (toggleOverlay == 1) {
            return input;
        } else{
            return original;
        }
    }

    // find the difference between two colors
    public double color_difference(Scalar color1, List<Integer> color2){
        double r1 = color1.val[0];
        double g1 = color1.val[1];
        double b1 = color1.val[2];

        int r2 = (int)color2.get(0);
        int g2 = (int)color2.get(1);
        int b2 = (int)color2.get(2);

        return Math.sqrt(Math.pow((r1 - r2), 2) + Math.pow((g1 - g2), 2) + Math.pow((b1 - b2), 2));
    }

    public void setAlliance(Alliance alliance){
        if (alliance == Alliance.RED) {
            PROP_COLOR = Arrays.asList(255, 0, 0);
        } else {
            PROP_COLOR = Arrays.asList(0, 0, 255);
        }
    }

    public Location getLocation(){
        return location;
    }

    public double getMinDifference(){
        return min_difference;
    }

    public void toggleOverlay(){
        toggleOverlay *= -1;
    }

    public boolean isOverlayShown() { return toggleOverlay == 1; }
}
