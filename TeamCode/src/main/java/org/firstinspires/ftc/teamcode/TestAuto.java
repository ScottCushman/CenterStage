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

            armDrive(.3, .3, 15, 2, collection, driveTrain);
            driveTrain.turnToPID(90, 2);
            driveTrain.encoderDrive(.3, 4, 2);

            driveTrain.encoderDrive(.3, -10, 2);
            driveTrain.turnToPID(-90, 3);
            driveTrain.encoderDrive(.3, 12, 2);
            driveTrain.strafeEncoderDrive(.3, 4.5, 2);
            driveTrain.encoderDrive(.1, 2, 2);
            collection.rotateArm(.86, 3);
            sleep(1000);
            armDrive(.2, .2, -3, 2, collection, driveTrain);
            driveTrain.strafeToDistanceSensorBackwards(.3, 5, 3);
            driveTrain.strafeEncoderDrive(.3, 5, 3);
            driveTrain.turnToPID(-90, 2);
            driveTrain.encoderDrive(.4, -50, 3);

            // Something with collection arm I don't have a clue
            collection.moveClaw(.3, 2);
            //Maybe something like this, I still need to finish it
            driveTrain.encoderDrive(.3, 48, 2);
            driveTrain.turnToPID(-90, 2);
            driveTrain.strafeToDistanceSensor(-.3, 8, 3);
            driveTrain.driveToDistanceSensor(.3, 3, 3);
            collection.rotateArm(.86, 3);

        } else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {

            driveTrain.encoderDrive(.1, 15, 2);
            driveTrain.encoderDrive(.1, -3, 2);
            driveTrain.turnToPID(-90, 2);
            driveTrain.encoderDrive(.1, 30, 2);
            driveTrain.strafeEncoderDrive(.1, 6.2, 2);
            driveTrain.encoderDrive(.07, 3.1, 1);
            collection.rotateArm(.86, 2);
            sleep(1500);
            driveTrain.encoderDrive(.2, -4, 2);
            collection.rotateArm(.2, 3);
            driveTrain.strafeEncoderDrive(.2, -25, 2);
            driveTrain.encoderDrive(.1, 10, 2);
        } else if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {

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
}