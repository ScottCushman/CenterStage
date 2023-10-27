package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */
public class SpikeMarkDetection {
    public enum spikeMarkPositions {
        LEFT,
        MIDDLE,
        RIGHT
    }
    private volatile SpikeMarkDetection.spikeMarkPositions position = spikeMarkPositions.LEFT;
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;
    private LinearOpMode theOpMode;

    public SpikeMarkDetection(HardwareMap hardwareMap, LinearOpMode opMode) {
        theOpMode = opMode;
    }


    public String detectPosition() {
       String position = spikeMarkPositions.LEFT;

        int cameraMonitorViewId = theOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", theOpMode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(theOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        while (theOpMode.opModeInInit()) {
            theOpMode.telemetry.addData("Can you see this?", 0);
            theOpMode.telemetry.update();
            if (SkystoneDeterminationPipeline.square1Activated) {
                position = SpikeMarkDetection.spikeMarkPositions.MIDDLE;
            }
            else if (SkystoneDeterminationPipeline.square2Activated) {
                position = SpikeMarkDetection.spikeMarkPositions.RIGHT;
            }
            else {
                position = SpikeMarkDetection.spikeMarkPositions.LEFT;
            }
        }
        return position;
    }


}