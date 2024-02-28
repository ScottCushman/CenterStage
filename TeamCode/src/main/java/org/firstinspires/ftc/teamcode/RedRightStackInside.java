package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous
public class RedRightStackInside extends LinearOpMode {

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
        waitForStart();
        if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {

            // collectArmDrive(.3,-6 , -.4, .3, 9.5, 3, collection, driveTrain);
            driveTrain.encoderDrive(.3, 11.6, 2);
            collection.collectionArm(-100, -.4, 3);
            //  collection.rotateArm(.3, 2);
            driveTrain.turnToPID(85, 2);
            // driveTrain.encoderDrive(.3, 6, 2);
            armDrive(.3, .3, 4.5, 2, collection, driveTrain);
            sleep(500);
            driveTrain.encoderDrive(.3, -11, 2);
            driveTrain.turnToPID(-89, 2);
            driveTrain.encoderDrive(.3, 8.2, 2);
            sleep(500);
            driveTrain.strafeEncoderDrive(.3, 16.5, 3);
            //collectArmDrive(-6, -.5, .3, 12, 2, collection, driveTrain);
            driveTrain.encoderDrive(.3, 3, 2);
            collection.rotateArm(.86, 3);
            sleep(800);
            armDrive(.209, .2, -3, 2, collection, driveTrain);
            driveTrain.turnToPID(-90, 1);
            driveTrain.strafeToDistanceSensorBackwards(.49, 9, 3);
            driveTrain.strafeEncoderDrive(.45, 8, 3);
            driveTrain.turnToPID(-89.5, 1);
            driveTrain.encoderDrive(.3, 10, 2);



        }
        else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {
            //driveTrain.encoderDrive(.3, 15.5, 2);
            armDrive(.2, .3, 15.5, 2, collection, driveTrain);
            sleep(500);
            armDrive(.2, .3, -5.2, 2, collection, driveTrain);
            driveTrain.strafeEncoderDrive(.3, -5, 2);
            driveTrain.turnToPID(-90, 1.5);
            collection.collectionArm(-30, .4, 1);
            driveTrain.encoderDrive(.3, 18, 2);
            sleep(500);
            driveTrain.strafeEncoderDrive(.4, 15, 2);
            sleep(500);
            driveTrain.encoderDrive(.3, 3, 2);
            collection.rotateArm(.86, 3);
            sleep(1000);
            armDrive(.209, .3, -2, 2, collection, driveTrain);
            driveTrain.turnToPID(-90, 1);
            driveTrain.strafeToDistanceSensorBackwards(.49, 9, 3);
            driveTrain.strafeEncoderDrive(.45, 2, 3);
            driveTrain.encoderDrive(.4, 7, 2);
            driveTrain.turnToPID(-89.5, 1);


        }
        else if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {

            driveTrain.encoderDrive(.4, 6.5, 2);
            driveTrain.strafeEncoderDrive(.4, -12, 3);
            driveTrain.encoderDrive(.4, 8, 2);
            collection.collectionArm(-135, -.4, 3);
            collection.rotateArm(.3, 2);
            driveTrain.encoderDrive(.3, -5.5, 2);
            driveTrain.turnToPID(-82, 1.5);
            driveTrain.encoderDrive(.3, 15, 2);
            sleep(500);
            driveTrain.turnToPID(-90, 1);
            driveTrain.strafeEncoderDrive(.3, 7, 2);
            driveTrain.encoderDrive(.3, 2, 2);
            collection.rotateArm(.86, 3);
            sleep(800);
            armDrive(.209, .3, -2, 2, collection, driveTrain);
            driveTrain.turnToPID(-90, 1);
            driveTrain.strafeEncoderDrive(.45, -15, 3);
            driveTrain.encoderDrive(.4, 14, 2);
            driveTrain.turnToPID(-89.5, 1);
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
}