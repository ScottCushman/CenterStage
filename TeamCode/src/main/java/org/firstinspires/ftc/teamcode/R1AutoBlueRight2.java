package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
@Disabled

@Autonomous
public class R1AutoBlueRight2 extends LinearOpMode {

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
        waitForStart();
        if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {
            sleep(7000);

            driveTrain.encoderDrive(.3, 11.5, 2);
            collection.collectionArm(-135, -.4, 3);
            //  collection.rotateArm(.3, 2);
            driveTrain.turnToPID(85, 2);
            // driveTrain.encoderDrive(.3, 6, 2);
            armDrive(.3, .3, 6, 2, collection, driveTrain);
            driveTrain.encoderDrive(.3, -4, 2);
            //  driveTrain.turnToPID(-90, 3);
            driveTrain.strafeEncoderDrive(.4, -32, 3);
            //   driveTrain.strafeEncoderDrive(.07, -2, 2);
            // driveTrain.turnToPID(-92, 1);
            driveTrain.turnToPID(90, 2);

            driveTrain.encoderDrive(.4, 60, 2);
            sleep(1000);
            driveTrain.turnToPID(90, 1.2);
            driveTrain.strafeToDistanceSensor(.4, 10, 3);
            driveTrain.strafeEncoderDrive(.3, 11, 3);
            driveTrain.turnToPID(90, 1.1);
            driveTrain.driveToDistanceSensor(.18, 2, 3);
            collection.rotateArm(.86, 3);
            sleep(800);
            armDrive(.3, .3, -4, 3, collection, driveTrain);

        }

        else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {
            sleep(7000);
            driveTrain.strafeEncoderDrive(.3, -3, 2);
            sleep(500);
            driveTrain.encoderDrive(.3, 19, 2);
            driveTrain.turnToPID(90, 2);
            collection.collectionArm(-135, -.4, 3);
            collection.rotateArm(.3, 2);
            driveTrain.encoderDrive(.3, -3, 2);

            driveTrain.strafeEncoderDrive(.2, -15, 3);
            //   driveTrain.strafeEncoderDrive(.07, -2, 2);
            // driveTrain.turnToPID(-92, 1);
            driveTrain.turnToPID(90, 2);

            driveTrain.encoderDrive(.4, 56, 2);
            sleep(1000);
            driveTrain.turnToPID(90, 1.2);
            driveTrain.strafeToDistanceSensor(.4, 10, 3);
            driveTrain.strafeEncoderDrive(.4, 4, 2);
            driveTrain.turnToPID(90, 1.1);
            driveTrain.driveToDistanceSensor(.2, 4, 3);
            collection.rotateArm(.86, 3);
            sleep(800);
            collection.rotateArm(.25, 3);
            sleep(800);
            armDrive(.3, .3, -4, 3, collection, driveTrain);
        }
        else if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {
            sleep(7000);

            driveTrain.encoderDrive(.4, 6.5, 2);
            driveTrain.strafeEncoderDrive(.4, -12.3, 3);
            driveTrain.encoderDrive(.4, 8, 2);
            collection.collectionArm(-135, -.4, 3);
            collection.rotateArm(.3, 2);
            driveTrain.encoderDrive(.3, -3, 2);
            driveTrain.strafeEncoderDrive(.4,11.1,2);
         //   driveTrain.
           // driveTrain.encoderDrive();
            driveTrain.encoderDrive(.4, 19.5, 2);
            driveTrain.turnToPID(90, 2);

//            driveTrain.strafeEncoderDrive(.2, 8.3, 2);
//            //   driveTrain.strafeEncoderDrive(.07, -2, 2);
//            // driveTrain.turnToPID(-92, 1);
//            driveTrain.turnToPID(90, 2);

            driveTrain.encoderDrive(.4, 50, 2);
            sleep(1000);
            driveTrain.turnToPID(90, 1.2);
            driveTrain.strafeToDistanceSensor(.4, 10, 3);
            driveTrain.strafeEncoderDrive(.4, -3, 2);
            driveTrain.turnToPID(90, 1.1);
            driveTrain.driveToDistanceSensor(.2, 4, 3);
            collection.rotateArm(.86, 3);
            sleep(800);
            collection.rotateArm(.25, 3);
            sleep(800);
            armDrive(.3, .3, -4, 3, collection, driveTrain);
        }


//        driveTrain.turnToPID(90, 2);
//        sleep(1000);
//        driveTrain.turnToPID(0, 2);

    }

    public void driveScan(double speed, double inches, double timeoutS, Drivetrain givenDriveTrain, Scanner givenScanner) {
        givenDriveTrain.strafeEncoderDriveStart(speed, inches, timeoutS);
        givenScanner.scanSignal();
        while(opModeIsActive() && givenDriveTrain.strafeEncoderDriveCheck(speed, inches, timeoutS)) {
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

    public void armClawLift(double armPos, double position, double power, int target, double timeoutS, Collection givenCollection, Lift givenLift) {
        givenCollection.rotateArmStart(armPos, timeoutS);
        givenCollection.rotateClawStart(position, timeoutS);
        givenLift.liftAutoStart(power, target, timeoutS);
        while(opModeIsActive() && givenCollection.rotateArmCheck(position, timeoutS) || (givenCollection.rotateClawCheck(position, timeoutS)) ||
                givenLift.liftAutoCheck(power, target, timeoutS));
    }
    public void rotateClaw(double armPos, double position, double timeoutS, Collection givenCollection) {
        givenCollection.rotateArmStart(armPos, timeoutS);
        givenCollection.rotateClawStart(position, timeoutS);
        while(opModeIsActive() && givenCollection.rotateArmCheck(armPos, timeoutS) || (givenCollection.rotateClawCheck(position, timeoutS)));
    }
    public void armDrive(double armPos, double speed, double inches, double timeoutS, Collection givenCollection, Drivetrain givenDrivetrain) {
        givenCollection.rotateArmStart(armPos, timeoutS);
        givenDrivetrain.encoderDriveStart(speed, inches, timeoutS);
        while (opModeIsActive() && givenCollection.rotateArmCheck(armPos, timeoutS) || givenDrivetrain.encoderDriveCheck(speed, inches, timeoutS));
    }

}