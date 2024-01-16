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
        waitForStart();
        if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {

           // collectArmDrive(.3,-6 , -.4, .3, 9.5, 3, collection, driveTrain);
            driveTrain.encoderDrive(.3, 9.5, 2);
            collection.collectionArm(-135, -.4, 3);
          //  collection.rotateArm(.3, 2);
            driveTrain.turnToPID(85, 2);
           // driveTrain.encoderDrive(.3, 6, 2);
            armDrive(.3, .3, 6, 2, collection, driveTrain);
            driveTrain.encoderDrive(.3, -11, 2);
            driveTrain.turnToPID(-89, 2);
            driveTrain.encoderDrive(.3, 8.2, 2);
            driveTrain.strafeEncoderDrive(.3, 12, 3);
            //collectArmDrive(-6, -.5, .3, 12, 2, collection, driveTrain);
            driveTrain.encoderDrive(.3, 3, 2);
            collection.rotateArm(.86, 3);
            sleep(800);
            armDrive(.209, .2, -3, 2, collection, driveTrain);
            driveTrain.turnToPID(-90, 1);
            driveTrain.strafeToDistanceSensorBackwards(.49, 9, 3);
            driveTrain.strafeEncoderDrive(.45, 8, 3);
            driveTrain.turnToPID(-89.5, 1);


            driveTrain.encoderDrive(.4, -69, 5);
            sleep(900);
            driveTrain.turnToPID(-79, 1.5);
            driveTrain.encoderDrive(.4, -3, 1);
            driveTrain.turnToPID(-89.5, 1);
            collection.collectionArm(29,.4, 1.2);

            collection.moveClaw(.2, 1);
            lift.liftAuto(.6, 120, 2);
            collection.collectionArm(165, .2, 1.5);
            collection.moveClaw(.01, 1);
            collection.collectionArm(-130, -.4, 1.4);
            armDrive(.3, .35, 68, 1.7, collection, driveTrain);
            sleep(500);
            driveTrain.turnToPID(-90, 2);
            driveTrain.strafeEncoderDrive(.3, -25, 3.2);
            driveTrain.turnToPID(-90, 2);
            driveTrain.driveToDistanceSensor(.15, 2.2, 3);
            collection.rotateArm(.86, 2);
            sleep(800);
            armDrive(.3, .2, -3, 2, collection, driveTrain);
            sleep(1000);
            collection.collectionArm(-90, -.4, 1);

        }
        else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {


        }
        else if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {

            driveTrain.encoderDrive(-.1, 13.8, 2);
            driveTrain.strafeEncoderDrive(.2, -7.2, 2);
            driveTrain.encoderDrive(.1, -4.8, 2);
            driveTrain.turnToPID(-90, 1.5);
            driveTrain.encoderDrive(.1, 16, 2);
            // driveTrain.strafeEncoderDrive(.1, 12, 3);

            collection.rotateArm(.86, 3);
            sleep(2000);
            driveTrain.encoderDrive(.1, -3.5, 2);
            collection.rotateArm(.2, 3);
            driveTrain.strafeEncoderDrive(.2, -20, 2);
            driveTrain.encoderDrive(.1, 8, 2);
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