package org.firstinspires.ftc.teamcode.Auto.TensorFlow;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Auto.Alliance;
import org.firstinspires.ftc.teamcode.Auto.Location;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class TensorFlowObjectDetector {
    private static final String MODEL_ASSET = "blue_red_prop_model_2.tflite";
    private static final String[] LABELS = { "blueProp", "redProp" };
    private static final float MIN_CONFIDENCE = .4f;
    private static final double CENTER_DIVISION = 350;
    private static final double MAX_SIZE = 200;
    private TfodProcessor tfod; // stores instance of TFOD processor
    public VisionPortal visionPortal; // stores instance of vision portal
    private Recognition mostConfidentRecognition, previousRecognition;
    public Alliance alliance;
    public TensorFlowObjectDetector(HardwareMap hardwareMap) {
        tfod = new TfodProcessor.Builder() // create the TF processor using a builder
                .setModelAssetName(MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder(); // create vision portal using a builder
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")); // set the camera
        builder.addProcessor(tfod); // set and enable processor
        visionPortal = builder.build(); // build the vision portal using the above settings
        tfod.setMinResultConfidence(MIN_CONFIDENCE); // set confidence threshold for TFOD recognitions
    }
    public void update() {
        List<Recognition> recognitions = tfod.getRecognitions();

        if(recognitions.size() != 0) {
            mostConfidentRecognition = recognitions.get(0);
            if(mostConfidentRecognition.getWidth() > MAX_SIZE || mostConfidentRecognition.getHeight() > MAX_SIZE) { // if invalid
                if(recognitions.size() > 1) { // set to next one if possible
                    mostConfidentRecognition = recognitions.get(1);
                } else // if no others set to null
                    mostConfidentRecognition = null;
            }
            if(mostConfidentRecognition != null) { // if not null must be valid
                float maxConfidence = mostConfidentRecognition.getConfidence();
                for (Recognition recognition : recognitions) {
                    if (recognition.getConfidence() > maxConfidence && recognition.getWidth() < MAX_SIZE && recognition.getHeight() < MAX_SIZE)
                        mostConfidentRecognition = recognition;
                }
                previousRecognition = mostConfidentRecognition;
            }
        } else {
            mostConfidentRecognition = null;
        }
    }
    public List<Recognition> getRecognitions() {
        return tfod.getRecognitions();
    }
    public Location getLocation() {
        return getLocation(mostConfidentRecognition);
    }
    public Location getLocation(Recognition recognition)
    {
        // if it doesn't see anything it must be left
        if(recognition == null)
            return Location.LEFT;

        double centerX = (recognition.getLeft() + recognition.getRight()) / 2;
        if(centerX > CENTER_DIVISION)
            return Location.RIGHT;
        else
            return Location.CENTER;
    }
    // sends info to telemetry about all found objects
    public void telemetryAll(Telemetry telemetry) {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        if(currentRecognitions.size() == 0)
            telemetry.addLine("No objects detected");
        else
            telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            telemetry.addLine();
            telemetrySingle(telemetry, recognition);
        }
        if(currentRecognitions.size() == 0 && previousRecognition != null) {
            telemetry.addLine("\nPreviously seen:");
            telemetrySingle(telemetry, previousRecognition);
        }
    }
    // sends info to telemetry about the single best object
    public void telemetryBest(Telemetry telemetry) {
        if(mostConfidentRecognition != null) {
            telemetrySingle(telemetry, mostConfidentRecognition);
        } else {
            telemetry.addLine("no objects found");
        }
    }
    public void telemetrySingle(Telemetry telemetry, Recognition recognition) {
        double x = (recognition.getLeft() + recognition.getRight()) / 2;
        double y = (recognition.getTop() + recognition.getBottom()) / 2;

        String line = String.format("%s found (%.0f %% conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
        if((alliance == Alliance.BLUE && !recognition.getLabel().equals("blueProp")) || (alliance == Alliance.RED && !recognition.getLabel().equals("redProp")))
            line += " (wrong color)";
        telemetry.addLine(line);
        telemetry.addData("- Position", "%s (%.0f, %.0f)", getLocation(recognition), x, y);
        String sizeLine = String.format("- Size: %.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        if(recognition.getWidth() > MAX_SIZE || recognition.getHeight() > MAX_SIZE)
            sizeLine += " (too big)";
        telemetry.addLine(sizeLine);
    }
    public void stopDetecting() { // save CPU resources when camera is no longer needed
        visionPortal.close();
        tfod.shutdown();
    }
}
