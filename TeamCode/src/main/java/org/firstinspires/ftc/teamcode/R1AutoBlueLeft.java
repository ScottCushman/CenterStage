package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous
public class R1AutoBlueLeft extends LinearOpMode {

    private int codePosition;

    public void runOpMode() {
        String tempString = "";
        ArrayList<Double> liftHeights = new ArrayList<Double>();
        liftHeights.add(3.0);
        liftHeights.add(16.5);
        liftHeights.add(3.0);
        liftHeights.add(16.5);
        Lift lift = new Lift(hardwareMap, this, 537.5, 1, 2, liftHeights);
        Scanner scanner = new Scanner(hardwareMap, this);
        Drivetrain driveTrain = new Drivetrain(hardwareMap, this, 537.6, 1.0, 4.0);
        Collection collection = new Collection(hardwareMap, this);
        SpikeMarkDetection spikeMarkDetection = new SpikeMarkDetection(hardwareMap, this);
        int counter = 0;
        spikeMarkDetection.detectPosition();
        waitForStart();
        if (spikeMarkDetection.detectPosition() == SpikeMarkDetection.spikeMarkPositions.LEFT) {
            driveTrain.diagonalDriveLeft(.4, 20, 3);
            driveTrain.encoderDrive(.4, 5, 3);
            driveTrain.turnToPID(90, 1);
            driveTrain.encoderDrive(-.6, 30, 3);
            codePosition = scanner.scanSignal();
            if (codePosition == 4) {
                lift.liftAuto(.5, 3, 3);
            }
            else {
                driveTrain.strafeEncoderDrive(.5, 8, 3);
            }
            codePosition = scanner.scanSignal();
            if (codePosition == 5) {

            }


        }
        else if (spikeMarkDetection.detectPosition() == SpikeMarkDetection.spikeMarkPositions.MIDDLE) {
            driveTrain.encoderDrive(.5, 20, 3);
        }
        else if (spikeMarkDetection.detectPosition() == SpikeMarkDetection.spikeMarkPositions.RIGHT) {
            driveTrain.diagonalDriveRight(.4, 20, 3);
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
    public void armClawLift(double armPos, double position, double speed, double distance, double timeoutS, Collection givenCollection, Lift givenLift) {
        givenCollection.rotateArmStart(armPos, timeoutS);
        givenCollection.rotateClawStart(position, timeoutS);
        givenLift.liftAutoStart(speed, distance, timeoutS);
        while(opModeIsActive() && givenCollection.rotateCollectionCheck(position, timeoutS) || (givenCollection.rotateClawCheck(position, timeoutS)) ||
        givenLift.liftAutoCheck(speed, distance, timeoutS));
    }
    public void rotateClaw(double position, double timeoutS, Collection givenCollection) {
        givenCollection.rotateCollectionStart(position, timeoutS);
        givenCollection.rotateClawStart(position, timeoutS);
        while(opModeIsActive() && givenCollection.rotateCollectionCheck(position, timeoutS) || (givenCollection.rotateClawCheck(position, timeoutS)));


    }
}