//this is just an outline. nothing has been tested.
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous
public class RedLeftInside extends LinearOpMode {

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

        //everything is just an outline.

        waitForStart();

        if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {
            driveTrain.awDrive(.1, .85, .1, .85, -13, 2);
            sleep(1000);
            driveTrain.encoderDrive(.1,-5,2);
            driveTrain.encoderDrive(.1,13.5,3);
            driveTrain.turnToPID(270,2);
            driveTrain.encoderDrive(.1,-45,2);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,-14,2);
            driveTrain.turnToPID(88,2);
            driveTrain.encoderDrive(.1,0.1,2);
            driveTrain.driveToDistanceSensor(.25,3,4);
            liftBox(.8, 500, .84, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(250);
            liftBox(-.3, -300, .55, 2, lift, collection);
            driveTrain.encoderDrive(.1,-4,2);
            sleep(250);
            liftBox(-.3, -300, .55, 2, lift, collection);
        }
        else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {
            driveTrain.encoderDrive(.2,-22,3);
            driveTrain.encoderDrive(.2,20,2);
            driveTrain.turnToPID(-270,2);
            driveTrain.encoderDrive(.1,-45,5);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,-9,2);
            driveTrain.turnToPID(90,2);
            driveTrain.encoderDrive(.1,0.1,2);
            driveTrain.driveToDistanceSensor(.25,2,4);
            liftBox(.8, 500, .84, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(250);
            liftBox(-.3, -300, .55, 2, lift, collection);
            driveTrain.encoderDrive(.1,-4,2);
            sleep(250);
            liftBox(-.3, -300, .55, 2, lift, collection);
        }
        if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {
            driveTrain.encoderDrive(.1,-10,3);
            driveTrain.turnToPID(-45,2);
            driveTrain.encoderDrive(.1,-8,2);
            driveTrain.encoderDrive(.1,6,2);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,9,2);
            driveTrain.turnToPID(270,2);
            driveTrain.encoderDrive(.25,-45,2);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,5,2);
            driveTrain.turnToPID(88,2);
            driveTrain.encoderDrive(.1,0.1,2);
            driveTrain.driveToDistanceSensor(.25,3,4);
            liftBox(.8, 500, .84, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(250);
            liftBox(-.3, -300, .55, 2, lift, collection);
            driveTrain.encoderDrive(.1,-4,2);
            sleep(250);
            liftBox(-.3, -300, .55, 2, lift, collection);
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