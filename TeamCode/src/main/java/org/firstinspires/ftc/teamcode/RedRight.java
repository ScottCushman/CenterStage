package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
@Autonomous
public class RedRight extends LinearOpMode {

    private int codePosition;
    private SpikeMarkDetection.spikeMarkPositions position;

    public void runOpMode() {
        String tempString = "";
        ArrayList<Double> liftHeights = new ArrayList<Double>();
        liftHeights.add(3.0);
        liftHeights.add(16.5);
        liftHeights.add(3.0);
        liftHeights.add(16.5);
        Lift lift = new Lift(hardwareMap, this, 537.5, 1, 2, liftHeights);
        //Scanner scanner = new Scanner(hardwareMap, this);
        Drivetrain driveTrain = new Drivetrain(hardwareMap, this, 145.1, 1.0, 4.0);
        Collection collection = new Collection(hardwareMap, this);
        SpikeMarkDetection spikeMarkDetection = new SpikeMarkDetection(hardwareMap, this);
        int counter = 0;
        position = spikeMarkDetection.detectPosition(true);
        collection.imAboutToDie.setPosition(.55);
        collection.imGoingToDie.setPosition(.55);

        waitForStart();

        if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {
            driveTrain.awDrive(.18, .4, .18, .4, -28, 2);
            driveTrain.awDrive(.55, 0.01, .55, 0.01, 17, 2);
            driveTrain.turnToPID(90, 2);
            driveTrain.encoderDrive(.4, 16, 2);

            liftBox(.8, 420, .84, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(1000);
            liftBox(-.3, -420, .58, 1.5, lift, collection);
            driveTrain.encoderDrive(.3, -4, 2);
            driveTrain.strafeEncoderDrive(.8, -25, 3.5);
            driveTrain.turnToPID(90, 1);
            driveTrain.encoderDrive(.3, 8, 2);

        }

        else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {

            driveTrain.encoderDrive(.8, -18.5, 2);
            driveTrain.aDrive(.7, .7, .7, .7, .25);
            driveTrain.driveToColorSensor(-.13, true, 4);
            driveTrain.encoderDrive(.3, 3.5, 2);
            driveTrain.turnToPID(90,2);
            driveRotate(.8, 19, 0, 2, driveTrain, collection);
            driveTrain.aDrive(-.7, -.7, -.7, -.7, .2);
            driveTrain.driveToDistanceSensor(.2, 6, 2);
            liftBox(.8, 420, .84, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(1000);
            liftBox(-.3, -420, .58, 1.5, lift, collection);
            driveTrain.encoderDrive(.3, -4, 2);
            driveTrain.strafeEncoderDrive(.9, -34, 3.5);
            driveTrain.turnToPID(90, 1);
            driveTrain.encoderDrive(.3, 8, 2);

        }

        else if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {
            driveTrain.encoderDrive(.8, -14, 2);
            driveTrain.aDrive(.7, .7, .7, .7, .25);
            driveTrain.turnToPID(-35, 2);
            driveTrain.encoderDrive(.3, -5, 2);
            driveTrain.encoderDrive(.3, 4, 2);
            driveTrain.awDrive(.6, 0, .6, 0, 35, 2);
            driveTrain.turnToPID(120, 2);
            driveTrain.encoderDrive(.4, 15, 2);
            driveTrain.turnToPID(90, 2);
            driveTrain.driveToDistanceSensor(.25, 5, 2);
            liftBox(.8, 420, .84, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(1000);
            liftBox(-.3, -420, .58, 1.5, lift, collection);

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
