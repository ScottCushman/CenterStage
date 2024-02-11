package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous
public class R2BlueLeft extends LinearOpMode {

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
        position = spikeMarkDetection.detectPosition(false);
        collection.imAboutToDie.setPosition(.55);
        collection.imGoingToDie.setPosition(.55);

        waitForStart();
        if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {
            driveTrain.encoderDrive(.15, -17, 3);
            driveTrain.turnToPID(45, 2);
            driveTrain.encoderDrive(.3, -2, 2);
            driveTrain.encoderDrive(.3, 3, 2);
            driveTrain.turnToPID(0,2);
            driveTrain.strafeEncoderDrive(.8, -30, 3);
            driveTrain.turnToPID(-120, 4);
            driveTrain.encoderDrive(.4, 19, 2);
            driveTrain.turnToPID(-90, 2);
            driveTrain.encoderDrive(.3, 4, 2);
            liftBox(.8, 250, .8, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(1000);
            liftBox(-.3, 0, .55, 2, lift, collection);
            driveTrain.encoderDrive(.4, -4, 2);
            driveTrain.strafeEncoderDrive(.6, 38, 3);
            driveTrain.turnToPID(-90, 2);
            driveTrain.strafeEncoderDrive(.6, 20, 2);
        }
        else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {
            driveTrain.encoderDrive(.1, -28, 3);
            driveTrain.encoderDrive(.1,3,2);
            driveTrain.strafeEncoderDrive(.7,-15,2);
            //driveTrain.awDrive(-.7, .7, -.7, .9, 20, 2);
            driveTrain.turnToPID(-110, 4);
            sleep(250);
            driveTrain.encoderDrive(.4,21,2);
            driveTrain.turnToPID(-90, 2);
            liftBox(.8, 250, .8, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(1000);
            liftBox(-.3, 0, .55, 2, lift, collection);
            driveTrain.encoderDrive(.4, -4, 2);
            driveTrain.turnToPID(2, 4);
            driveTrain.encoderDrive(.5, 17, 3);
        }
        if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {
            driveTrain.encoderDrive(.1, -8, 3);
            sleep(250);
            driveTrain.awDrive(.6, .15, .6, .15, -20, 2);
            driveTrain.turnToPID(-60,3);
            driveTrain.encoderDrive(.15, -4, 2);
            driveTrain.encoderDrive(.3,7,2);
            driveTrain.turnToPID(-110,3);
            driveTrain.encoderDrive(.4,24,2);
            driveTrain.turnToPID(-90, 2);
            driveTrain.encoderDrive(.3, 2, 2);
            liftBox(.8, 250, .8, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(1000);
            liftBox(-.3, 0, .55, 2, lift, collection);
            driveTrain.encoderDrive(.4, -4, 2);
            driveTrain.turnToPID(0, 4);
            driveTrain.encoderDrive(.5, 13, 3);


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
    public void liftBox(double power, int target, double armPos, double timeoutS, Lift givenLift, Collection givenCollection) {
        givenLift.liftAutoStart(power, target, timeoutS);
        givenCollection.rotateArmStart(armPos, timeoutS);
        while (opModeIsActive() && givenLift.liftAutoCheck(power, target, timeoutS) || givenCollection.rotateArmCheck(armPos, timeoutS));
    }
    public void driveCollect(double speed, double inches, double power, double rotations, double timeoutS, Drivetrain givenDrivetrain, Collection givenCollection) {
        givenDrivetrain.encoderDriveStart(speed, inches, timeoutS);
        givenCollection.collectionStart(power, rotations, timeoutS);
        while (opModeIsActive() && givenDrivetrain.encoderDriveCheck(speed, inches, timeoutS) || givenCollection.collectionCheck(power, rotations, timeoutS));
    }
}