package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;

public class SpikeMarkDetection {
    public enum spikeMarkPositions {
        LEFT,
        MIDDLE,
        RIGHT,
        NOTHING
    }
    String something;
    private volatile SpikeMarkDetection.spikeMarkPositions positions;
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;
    private LinearOpMode theOpMode;

    public SpikeMarkDetection(HardwareMap hardwareMap, LinearOpMode opMode) {
        theOpMode = opMode;
    }


    public spikeMarkPositions detectPosition() {
       spikeMarkPositions positions = spikeMarkPositions.NOTHING;

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
                positions = SpikeMarkDetection.spikeMarkPositions.MIDDLE;
            }
            else if (SkystoneDeterminationPipeline.square2Activated) {
                positions = SpikeMarkDetection.spikeMarkPositions.RIGHT;
            }
            else {
                positions = SpikeMarkDetection.spikeMarkPositions.LEFT;
            }
        }
        theOpMode.telemetry.addData("Hello?", 0);
        theOpMode.telemetry.update();
        return positions;
    }


}