package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous
public class R1AutoRedLeft extends LinearOpMode {

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
        if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {
            driveTrain.turnToPID(0, 2);
            sleep(8000);
            driveTrain.encoderDrive(.1, 8.2, 2);
            driveTrain.turnToPID(-90, 2);
            driveTrain.encoderDrive(.1, 2.5, 2);
            driveTrain.encoderDrive(.1, -4, 2);
            driveTrain.turnToPID(-90, 3);

            driveTrain.strafeEncoderDrive(.2, -15, 2);
            //   driveTrain.strafeEncoderDrive(.07, -2, 2);
            // driveTrain.turnToPID(-92, 1);
            driveTrain.turnToPID(-90, 2);

            driveTrain.encoderDrive(.4, 25, 2);
            sleep(1000);
            driveTrain.turnToPID(-90, 1.2);
            driveTrain.strafeEncoderDrive(.2, 12, 2);
            driveTrain.encoderDrive(.1, 9, 2);
            collection.rotateArm(.86, 3);
            sleep(2000);
            collection.rotateArm(.25, 3);
            sleep(2000);
            driveTrain.encoderDrive(.1, -2, 2);

        }

        else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {
            driveTrain.turnToPID(0, 2);
            sleep(8000);
            driveTrain.encoderDrive(.1, 15, 2);
            driveTrain.encoderDrive(.1, -3.8, 2);
            driveTrain.turnToPID(-90, 2);

            driveTrain.strafeEncoderDrive(.2, -8.2, 2);
            //   driveTrain.strafeEncoderDrive(.07, -2, 2);
            // driveTrain.turnToPID(-92, 1);
            driveTrain.turnToPID(-90, 2);

            driveTrain.encoderDrive(.4, 25, 2);
            sleep(1000);
            driveTrain.turnToPID(-90, 1.2);
            driveTrain.strafeEncoderDrive(.2, 16, 2);
            driveTrain.encoderDrive(.1, 9, 2);
            collection.rotateArm(.86, 3);
            sleep(2000);
            collection.rotateArm(.25, 3);
            sleep(2000);
            driveTrain.encoderDrive(.1, -2, 2);
        }
        else if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {
            driveTrain.turnToPID(0, 2);
            sleep(8000);
            driveTrain.encoderDrive(.1, 14, 2);
            driveTrain.strafeEncoderDrive(.2,9.2,2);
            driveTrain.encoderDrive(.1, -6, 2);
            driveTrain.turnToPID(-90, 2);

            driveTrain.strafeEncoderDrive(.2, -8.8, 2);
            //   driveTrain.strafeEncoderDrive(.07, -2, 2);
            // driveTrain.turnToPID(-92, 1);
            driveTrain.turnToPID(-90, 2);

            driveTrain.encoderDrive(.4, 28, 2);
            sleep(1000);
            driveTrain.turnToPID(-90, 1.2);
            driveTrain.strafeEncoderDrive(.2, 19, 2);
            driveTrain.encoderDrive(.1, 8, 2);
            collection.rotateArm(.86, 3);
            sleep(2000);
            collection.rotateArm(.25, 3);
            sleep(2000);
            driveTrain.encoderDrive(.1, -2, 2);
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