package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous
public class RedLeft extends LinearOpMode {

    private int codePosition;
    private SpikeMarkDetection.spikeMarkPositions position;

    public void runOpMode() {
        //  String tempString = "";
        ArrayList<Double> liftHeights = new ArrayList<Double>();
        liftHeights.add(3.0);
        liftHeights.add(16.5);
        liftHeights.add(3.0);
        liftHeights.add(16.5);
        Lift lift = new Lift(hardwareMap, this, 537.5, 1, 2, liftHeights);
        Scanner scanner = new Scanner(hardwareMap, this);
        Drivetrain driveTrain = new Drivetrain(hardwareMap, this, 145.1, 1, 4.0);
        Collection collection = new Collection(hardwareMap, this);
        SpikeMarkDetection spikeMarkDetection = new SpikeMarkDetection(hardwareMap, this);
        position = spikeMarkDetection.detectPosition(true);
        collection.imAboutToDie.setPosition(.55);
        collection.imGoingToDie.setPosition(.55);

        waitForStart();
        //delivery testing
        liftBox(.8, 500, .84, 2, lift, collection);
        collection.rotServo.setPosition(0);
        driveTrain.encoderDrive(.1,-4,2);
        liftBox(-.3, -300, .55, 2, lift, collection);
        sleep(250);
        liftBox(-.3, -300, .55, 2, lift, collection);

        /*
        //left detection
        driveTrain.awDrive(.15, .45, .15, .45, -15, 2);
        sleep(500);
        driveTrain.encoderDrive(.1,11.5,2);
        driveTrain.turnToPID(0,2);
        driveTrain.encoderDrive(.2,-28,2);
        driveTrain.turnToPID(270,2);
        driveTrain.encoderDrive(.25,-30,2);
        driveTrain.turnToPID(45,2);
        driveTrain.encoderDrive(.1,18,2);
        driveTrain.turnToPID(90,2);

         */
        /*
        driveTrain.encoderDrive(.25,-55,5);
        driveTrain.turnToPID(0,2);
        driveTrain.encoderDrive(.1,30,2);
        driveTrain.turnToPID(90,2);

         */

        sleep(123456);


        if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {
            driveTrain.awDrive(.45, .15, .45, .15, -15, 2);
            driveTrain.encoderDrive(.2,-13,3);
            driveTrain.turnToPID(45,2);
            driveTrain.encoderDrive(.2,-1.5,2);
            driveTrain.encoderDrive(.1,3,2);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.2,-12,2);
            driveTrain.turnToPID(90,2);
            driveTrain.encoderDrive(.25,55,5);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,11,2);
            driveTrain.turnToPID(270,2);
            driveTrain.encoderDrive(.1,4,2);
            liftBox(.8, 500, .84, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(1000);
            liftBox(-.3, -300, .55, 2, lift, collection);
            driveTrain.encoderDrive(.2,4,2);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,-16,2);
            driveTrain.turnToPID(270,2);
            driveTrain.encoderDrive(.1,10,2);
        }
        else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {
            driveTrain.awDrive(.45, .15, .45, .15, -15, 2);
            driveTrain.encoderDrive(.2,-17,3);
            driveTrain.encoderDrive(.2,-1.5,2);
            driveTrain.turnToPID(45,2);
            driveTrain.encoderDrive(.1,-10,2);
            driveTrain.turnToPID(-45,2);
            driveTrain.encoderDrive(.1,-10,2);
            driveTrain.turnToPID(90,2);
            driveTrain.encoderDrive(.25,55,5);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,14,2);
            driveTrain.turnToPID(270,2);
            driveTrain.encoderDrive(.1,4,2);
            liftBox(.8, 500, .84, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(1000);
            liftBox(-.3, -300, .55, 2, lift, collection);
            driveTrain.encoderDrive(.2,4,2);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,-19,2);
            driveTrain.turnToPID(270,2);
            driveTrain.encoderDrive(.1,10,2);


        }
        if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {
            driveTrain.awDrive(.45, .15, .45, .15, -15, 2);
            driveTrain.encoderDrive(.2,-13,3);
            driveTrain.turnToPID(-45,2);
            driveTrain.encoderDrive(.2,-1.5,2);
            driveTrain.encoderDrive(.1,3,2);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.2,-12,2);
            driveTrain.turnToPID(90,2);
            driveTrain.encoderDrive(.25,55,5);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,17,2);
            driveTrain.turnToPID(270,2);
            driveTrain.encoderDrive(.1,4,2);
            liftBox(.8, 500, .84, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(1000);
            liftBox(-.3, -300, .55, 2, lift, collection);
            driveTrain.encoderDrive(.2,4,2);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,-22,2);
            driveTrain.turnToPID(270,2);
            driveTrain.encoderDrive(.1,10,2);


        }

    }


    public void liftBox(double power, int target, double armPos, double timeoutS, Lift givenLift, Collection givenCollection) {
        givenLift.liftAutoStart(power, target, timeoutS);
        givenCollection.rotateArmStart(armPos, timeoutS);
        while (opModeIsActive() && givenLift.liftAutoCheck(power, target, timeoutS) || givenCollection.rotateArmCheck(armPos, timeoutS));
    }
    public void driveRotate(double speed, double inches, double clawPos, double timeoutS, Drivetrain givenDrivetrain, Collection givenCollection) {
        givenDrivetrain.encoderDriveStart(speed, inches, timeoutS);
        givenCollection.moveClawStart(clawPos, timeoutS);
        while (opModeIsActive() && givenDrivetrain.encoderDriveCheck(speed, inches, timeoutS) || givenCollection.moveClawCheck(clawPos, timeoutS));
    }
    public void driveRotateSpin(double speed, double inches, double clawPos, double power, double rotations, double timeoutS, Drivetrain givenDrivetrain, Collection givenCollection) {
        givenDrivetrain.encoderDriveStart(speed, inches, timeoutS);
        givenCollection.moveClawStart(clawPos, timeoutS);
        givenCollection.collectionStart(power, rotations, timeoutS);
        while (opModeIsActive() && givenDrivetrain.encoderDriveCheck(speed, inches, timeoutS) || givenCollection.moveClawCheck(clawPos, timeoutS) || givenCollection.collectionCheck(power, rotations, timeoutS));
    }
}
