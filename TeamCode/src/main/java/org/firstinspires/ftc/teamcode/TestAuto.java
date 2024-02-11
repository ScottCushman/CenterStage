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
        collection.rotServo.setPosition(0);
        collection.imAboutToDie.setPosition(.55);
        collection.imGoingToDie.setPosition(.55);
        waitForStart();
        if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {
            driveTrain.encoderDrive(.4, -23, 2);
            driveTrain.turnToPID(145, 2);
            //driveTrain.encoderDrive(.4, -6, 2);
            sleep(200);
            driveTrain.encoderDrive(.3, -5, 2);
            driveCollect(.4, 5 ,.3, 200, 2, driveTrain, collection);
            driveTrain.turnToPID(90, 2);
            driveRotateSpin(.2, -20, .75,-.8, 100, 2,driveTrain,collection);
            sleep(500);
            driveRotateSpin(.4, 15, .75,-.8, 100, 2,driveTrain,collection);
            collection.openBox(.22, 2);
            sleep(800);
            driveTrain.encoderDrive(.4, 50, 2);
            driveTrain.turnToPID(35, 2);
            driveTrain.encoderDrive(.4, 20, 2);
            driveTrain.turnToPID(90, 2);
            driveTrain.encoderDrive(.2, 1.5, 2);
            liftBox(.5, 350, .8, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(1000);
            }
            /*
            driveTrain.encoderDrive(.4, -10, 2);
            driveTrain.turnToPID(30, 2);
            //driveTrain.encoderDrive(.4, -6, 2);
            sleep(200);
            driveCollect(.4, 4 ,.3, 200, 2, driveTrain, collection);
            driveTrain.turnToPID(0, 1);
            driveTrain.encoderDrive(.4, 8, 2);
            driveTrain.turnToPID(90.8, 2);
            driveTrain.encoderDrive(.4, 50, 3);
            driveTrain.turnToPID(140, 2);
            driveTrain.encoderDrive(.4, 22, 2);
            driveTrain.turnToPID(90, 2);
            driveTrain.encoderDrive(.2, 1.5, 2);
            liftBox(.5, 350, .8, 2, lift, collection);
            collection.rotServo.setPosition(.3);
            sleep(1000);            //driveTrain.


        }
        driveTrain.encoderDrive(.4, -10, 2);
        driveTrain.turnToPID(30, 2);
        //driveTrain.encoderDrive(.4, -6, 2);
        sleep(200);
        driveCollect(.4, 4 ,.3, 200, 2, driveTrain, collection);
        driveTrain.turnToPID(90, 2);
        driveRotateSpin(.4, -8, .75,-.8, 100, 2,driveTrain,collection);
        driveRotateSpin(.4, 8, .75,-.8, 100, 2,driveTrain,collection);




             */
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
    public void rotateDrive(int rotation, double power, double speed, double inches, double timeoutS, Collection givenCollection, Drivetrain givenDrivetrain) {
        givenCollection.collectionArmStart(rotation, power, timeoutS);
        givenDrivetrain.encoderDriveStart(speed, inches, timeoutS);
        while (opModeIsActive() && givenCollection.collectionArmCheck(rotation, power, timeoutS) || givenDrivetrain.encoderDriveCheck(speed, inches, timeoutS));
    }

     */
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