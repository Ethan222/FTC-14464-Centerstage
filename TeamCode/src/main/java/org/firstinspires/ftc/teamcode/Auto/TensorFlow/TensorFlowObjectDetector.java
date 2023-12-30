package org.firstinspires.ftc.teamcode.Auto.TensorFlow;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Auto.Location;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class TensorFlowObjectDetector {
    private static final String MODEL_ASSET = "blue_red_prop_model_1.tflite";
    private static final String[] LABELS = { "blueProp", "redProp" };
    private static final float MIN_CONFIDENCE = .5f;
    private static final double LEFT_CENTER_DIVISION = 200;
    private TfodProcessor tfod; // stores instance of TFOD processor
    public VisionPortal visionPortal; // stores instance of vision portal
    private Recognition mostConfidentRecognition, previousRecognition;
    public TensorFlowObjectDetector(HardwareMap hardwareMap) {
        tfod = new TfodProcessor.Builder() // create the TF processor using a builder
                .setModelAssetName(MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder(); // create vision portal using a builder
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")); // set the camera
        builder.addProcessor(tfod); // set and enable processor
        visionPortal = builder.build(); // build the vision portal using the above settings
        //tfod.setMinResultConfidence(MIN_CONFIDENCE); // set confidence threshold for TFOD recognitions
    }
    public void update() {
        List<Recognition> recognitions = tfod.getRecognitions();

        if(recognitions.size() != 0) {
            mostConfidentRecognition = recognitions.get(0);
            float maxConfidence = mostConfidentRecognition.getConfidence();
            for (Recognition recognition : recognitions) {
                if (recognition.getConfidence() > maxConfidence)
                    mostConfidentRecognition = recognition;
            }
            previousRecognition = mostConfidentRecognition;
        } else {
            mostConfidentRecognition = null;
        }
    }
    public List<Recognition> getRecognitions() {
        return tfod.getRecognitions();
    }
    public Location getLocation() {
        update();
        // if it doesn't see anything it must be right
        if(mostConfidentRecognition == null)
            return Location.RIGHT;

        double centerY = (mostConfidentRecognition.getTop() + mostConfidentRecognition.getBottom()) / 2;
        if(centerY > LEFT_CENTER_DIVISION)
            return Location.LEFT;
        else
            return Location.CENTER;
    }
    // sends info to telemetry about all found objects
    public void telemetryAll(Telemetry telemetry) {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            telemetry.addLine();
            telemetrySingle(telemetry, recognition);
        }
        if(previousRecognition != null) {
            telemetry.addLine("Previously seen:");
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
        double x = (mostConfidentRecognition.getLeft() + mostConfidentRecognition.getRight()) / 2;
        double y = (mostConfidentRecognition.getTop() + mostConfidentRecognition.getBottom()) / 2;

        telemetry.addLine(String.format("%s found (%.0f %% conf.)", mostConfidentRecognition.getLabel(), mostConfidentRecognition.getConfidence() * 100));
        telemetry.addData("- Position", "(%.0f, %.0f)", x, y);
        telemetry.addData("- Size", "%.0f x %.0f", mostConfidentRecognition.getWidth(), mostConfidentRecognition.getHeight());
    }
    public void stopDetecting() { // save CPU resources when camera is no longer needed
        visionPortal.close();
    }
}
