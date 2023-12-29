package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous
public class R1AutoRedRight extends LinearOpMode {

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
        Drivetrain driveTrain = new Drivetrain(hardwareMap, this, 537.6, .5, 4.0);
        Collection collection = new Collection(hardwareMap, this);
        SpikeMarkDetection spikeMarkDetection = new SpikeMarkDetection(hardwareMap, this);
     //   int counter = 0;
        position = spikeMarkDetection.detectPosition(true);
        waitForStart();
        if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {

            driveTrain.encoderDrive(.1, 8.5, 2);
            driveTrain.turnToPID(90, 2);
            driveTrain.encoderDrive(.1, 3, 2);
            driveTrain.encoderDrive(.1, -10, 2);
            driveTrain.turnToPID(-90, 3);
            driveTrain.encoderDrive(.1, 9, 2);
            driveTrain.strafeEncoderDrive(.1, 4.5, 2);
            driveTrain.encoderDrive(.1, 2, 2);
            collection.rotateArm(.86, 3);
            sleep(2000);
            driveTrain.encoderDrive(.1, -3, 2);
            collection.rotateArm(.2, 3);
            driveTrain.strafeEncoderDrive(.1, -22, 2);
            driveTrain.encoderDrive(.1, 6, 2);
           // armClawLift(.8, .1, .7, 700, 3, collection, lift);

           // lift.liftAuto(.7, 700, 4);
        }

        else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {

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
        }
        else if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {

            driveTrain.encoderDrive(-.1, 13.8, 2);
            driveTrain.strafeEncoderDrive(.2,-7.2,2);
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

}