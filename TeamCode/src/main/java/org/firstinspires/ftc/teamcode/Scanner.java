package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class Scanner {
    OpenCvWebcam webcam;
    private LinearOpMode theOpMode;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    AprilTagDetection tagOfInterest = null;

    public Scanner(HardwareMap hardwareMap, LinearOpMode opMode) {
        theOpMode = opMode;

    }

    public int scanSignal() {
        String signalPosition;
        int aprilID = 0;


        int cameraMonitorViewId = theOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                theOpMode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(theOpMode.hardwareMap.get(WebcamName.class,""),
                cameraMonitorViewId);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        webcam.setPipeline(aprilTagDetectionPipeline);
        //   SkystoneDeterminationPipeline skystoneDeterminationPipeline = new SkystoneDeterminationPipeline(tagsize, fx, fy, cx, cy);

        //    webcam.setPipeline(SkystoneDeterminationPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);

            }

            @Override
            public void onError(int errorCode) {
            }
        });
        signalPosition = "";
        int counter = 0;
        while ((theOpMode.opModeInInit())) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            currentDetections.size();
            theOpMode.telemetry.addData("April ID", aprilID);
            theOpMode.telemetry.update();
            if (currentDetections.size() > 0) {
                aprilID = currentDetections.get(0).id;
            }

        }
        return aprilID;
/*
            if (!aprilTagDetectionPipeline.codePosition.equals("")) {
                signalPosition = pipeline.codePosition;
                counter = counter + 1;
                counter = counter % 10000;
                theOpMode.telemetry.addData("Count = ", counter);
                theOpMode.telemetry.addData(signalPosition, null);
                theOpMode.telemetry.update();
            }
//           counter = counter + 1;
//           counter = counter % 10000;
//            theOpMode.telemetry.addData("Count = ", counter);
//            theOpMode.telemetry.update();
//          theOpMode.telemetry.addData("opMode inInit is %b", theOpMode.opModeInInit());
//          theOpMode.telemetry.addData(signalPosition, 3);
//          theOpMode.telemetry.update();
        }
        theOpMode.telemetry.addData(signalPosition, null);
        theOpMode.telemetry.update();
        //  theOpMode.sleep(4000);
//webcam.closeCameraDevice();
        return signalPosition;
    }


 */
    }
    public void detectColor() {
        //    Imgproc.
    }
}

        /*
    class ConvertToGreyPipeline extends OpenCvPipeline {
        // Notice this is declared as an instance variable (and re-used), not a local variable
        Mat grey = new Mat();
        QRCodeDetector qrCodeDetector = new QRCodeDetector();
        String codePosition = "";

        Mat contrast = new Mat();


        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
            Imgproc.threshold(grey, contrast, 122, 255, Imgproc.THRESH_BINARY);
            codePosition = qrCodeDetector.detectAndDecodeCurved(contrast);

            return contrast;

//Austin is stupid
        }

    }
        */



