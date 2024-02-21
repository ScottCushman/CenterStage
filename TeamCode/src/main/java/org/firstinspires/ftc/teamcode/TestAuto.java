package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous
public class TestAuto extends LinearOpMode {

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
        //   int counter = 0;
        position = spikeMarkDetection.detectPosition(true);
        collection.imAboutToDie.setPosition(.55);
        collection.imGoingToDie.setPosition(.55);
        waitForStart();
        if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {
          driveTrain.encoderDrive(.4, -10, 2);
          driveTrain.driveToRangeSensor(-.2, 10, 2);
        }
        else if (position == SpikeMarkDetection.spikeMarkPositions.MIDDLE) {
            driveTrain.encoderDrive(.8, -18, 2);
            driveTrain.aDrive(.7, .7, .7, .7, .2);
            driveTrain.driveToColorSensor(-.13, true, 4);
            driveTrain.encoderDrive(.3, 4, 2);
            driveTrain.turnToPID(90,2);
            driveRotate(.4, 21, 0, 2, driveTrain, collection);
            liftBox(.8, 500, .84, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(1000);
            liftBox(-.3, -420, .58, 2, lift, collection);
            driveTrain.encoderDrive(.4, -4, 2);
//            driveTrain.turnToPID(0, 2);
//            driveTrain.encoderDrive(.4, 15, 2);

            driveTrain.turnToPID(45, 2);
            driveRotate(.4, -18.8, .8, 3, driveTrain, collection);
            driveTrain.turnToPID(90, 2);
            driveTrain.encoderDrive(.4, -32, 3);
            sleep(500);
            driveTrain.driveToDistanceSensor(-.19, 6, .5);
            driveRotateSpin(.06, -4, .71, -1, 200, 3, driveTrain, collection);
            driveRotateSpin(.06, 3, .64, -1, 200, 3, driveTrain, collection);
            driveTrain.encoderDrive(.2, 2, 2);
            collection.collection(-1, 200, 2);
            collection.rotServo.setPosition(.3);
            sleep(500);
            driveRotateSpin(.4, 46, .55, .9, 300, 2, driveTrain, collection);
            sleep(500);
            driveTrain.turnToPID(45, 2);
            driveTrain.encoderDrive(.4, 18, 3);
            driveTrain.turnToPID(90, 2);
            driveTrain.encoderDrive(.3, 3, 2);
            liftBox(.8, 500, .84, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(1000);
            liftBox(-.3, -300, .55, 2, lift, collection);
        }
    }

    public void driveScan(double speed, double inches, double timeoutS, Drivetrain givenDriveTrain, Scanner givenScanner) {
        givenDriveTrain.strafeEncoderDriveStart(speed, inches, timeoutS);
        givenScanner.scanSignal();
        while (opModeIsActive() && givenDriveTrain.strafeEncoderDriveCheck(speed, inches, timeoutS)) {
            codePosition = givenScanner.scanSignal();
            if (codePosition >= 3) {
            }
        }
    }

    public void rotateArmClaw(double armPos, double position, double timeoutS, Collection givenCollection) {
        givenCollection.rotateArmStart(armPos, timeoutS);
        givenCollection.rotateClawStart(position, timeoutS);
        while (opModeIsActive() && givenCollection.rotateArmCheck(armPos, timeoutS) || (givenCollection.rotateClawCheck(position, timeoutS)));
    }

    public void armDrive(double armPos, double speed, double inches, double timeoutS, Collection givenCollection, Drivetrain givenDrivetrain) {
        givenCollection.rotateArmStart(armPos, timeoutS);
        givenDrivetrain.encoderDriveStart(speed, inches, timeoutS);
        while (opModeIsActive() && givenCollection.rotateArmCheck(armPos, timeoutS) || givenDrivetrain.encoderDriveCheck(speed, inches, timeoutS));
    }

    public void armStrafe(double armPos, double speed, double inches, double timeoutS, Collection givenCollection, Drivetrain givenDrivetrain) {
        givenCollection.rotateArmStart(armPos, timeoutS);
        givenDrivetrain.encoderDriveStart(speed, inches, timeoutS);
        while (opModeIsActive() && givenCollection.rotateArmCheck(armPos, timeoutS) || givenDrivetrain.strafeEncoderDriveCheck(speed, inches, timeoutS));
    }



    /*
    public void rotateStrafeSense(int rotation, double power, double APPROACH_SPEED, double inches, double timeoutS, Collection givenCollection, Drivetrain givenDrivetrain) {
        givenCollection.collectionArmStart(rotation, power, timeoutS);
        givenDrivetrain.strafeToDistanceSensorBackwardsStart(APPROACH_SPEED, inches, timeoutS);
        while (opModeIsActive() && givenCollection.collectionArmCheck(rotation, power, timeoutS) || givenDrivetrain.strafeToDistanceSensorBackwardsCheck(APPROACH_SPEED, inches, timeoutS));
    }

     */
    public void collectDrive(double clawPos, double speed, double inches, double timeoutS, Collection givenCollection, Drivetrain givenDrivetrain) {
        givenCollection.moveClawStart(clawPos, timeoutS);
        givenDrivetrain.encoderDriveStart(speed, inches, timeoutS);
        while (opModeIsActive() && givenCollection.moveClawCheck(clawPos, timeoutS) || givenDrivetrain.encoderDriveCheck(speed, inches, timeoutS));
    }
    public void collectArmDrive(double armPos, int rotation, double power, double speed, double inches, double timeoutS, Collection givenCollection, Drivetrain givenDrivetrain) {
        givenCollection.rotateArmStart(armPos, timeoutS);
        givenCollection.collectionArmStart(rotation, power, timeoutS);
        givenDrivetrain.encoderDriveStart(speed, inches, timeoutS);
        while (opModeIsActive() && givenCollection.rotateArmCheck(armPos, timeoutS) || givenCollection.collectionArmCheck(rotation, power, timeoutS) || givenDrivetrain.encoderDriveCheck(speed, inches,timeoutS));
    }
    public void driveCollect(double speed, double inches, double power, double rotations, double timeoutS, Drivetrain givenDrivetrain, Collection givenCollection) {
        givenDrivetrain.encoderDriveStart(speed, inches, timeoutS);
        givenCollection.collectionStart(power, rotations, timeoutS);
        while (opModeIsActive() && givenDrivetrain.encoderDriveCheck(speed, inches, timeoutS) || givenCollection.collectionCheck(power, rotations, timeoutS));
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