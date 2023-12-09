/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp(group = "test")
public class TestAprilTagDetection extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    AprilTagIDs ids = new AprilTagIDs();
    Backdrop backdrop = ids.blueBackdrop;

    AprilTagDetection[] detectedTags = {null, null, null};
    boolean[] currentlyDetecting = {false, false, false};

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        // init loop
        while (!isStarted() && !isStopRequested())
        {
            if(gamepad1.x) {
                backdrop = ids.blueBackdrop;
                detectedTags = new AprilTagDetection[3];
            }
            else if(gamepad1.b) {
                backdrop = ids.redBackdrop;
                detectedTags = new AprilTagDetection[3];
            }

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            currentlyDetecting = new boolean[3];
            if(currentDetections.size() != 0)
            {
                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == backdrop.left) {
                        detectedTags[0] = tag;
                        currentlyDetecting[0] = true;
                    } else if(tag.id == backdrop.center) {
                        detectedTags[1] = tag;
                        currentlyDetecting[1] = true;
                    } else if(tag.id == backdrop.right) {
                        detectedTags[2] = tag;
                        currentlyDetecting[2] = true;
                    }
                }
            }

            telemetrizeDetectedAprilTags();
            telemetry.update();
            sleep(20);
        }

        // prevent the opmode from ending
        while (opModeIsActive()) {sleep(20);}
    }

    private void telemetrizeDetectedAprilTags()
    {
        telemetry.addLine(backdrop.color + " backdrop");
        String[] strings = {
                "LEFT (tag " + backdrop.arr[0] + ")",
                "CENTER (tag " + backdrop.arr[1] + ")",
                "RIGHT (tag " + backdrop.arr[2] + ")"
        };
        for(int i = 0; i < 3; i++) {
            telemetry.addLine(strings[i]);
            AprilTagDetection tag = detectedTags[i];
            if(tag != null) {
                telemetry.addLine(currentlyDetecting[i] ? "currently detecting" : "not currently detecting");
                tagToTelemetry(tag);
            } else {
                telemetry.addLine("not found");
            }
            telemetry.addLine();
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        double x = detection.pose.x*FEET_PER_METER;
        double y = detection.pose.y*FEET_PER_METER;
        double z = detection.pose.z*FEET_PER_METER;
        telemetry.addLine(String.format("(%.2f, %.2f, %.2f)", x, y, z));
    }
}