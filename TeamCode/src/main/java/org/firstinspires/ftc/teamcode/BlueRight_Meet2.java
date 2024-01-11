package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous
public class BlueRight_Meet2 extends LinearOpMode {

    private int codePosition;
    private SpikeMarkDetection.spikeMarkPositions position;

    public void runOpMode() {
        String tempString = "";
        ArrayList<Double> liftHeights = new ArrayList<Double>();
        liftHeights.add(3.0);
        liftHeights.add(16.5);
        liftHeights.add(3.0);
        liftHeights.add(16.5);
        Lift lift = new Lift(hardwareMap, this, 145.1, 1, 2, liftHeights);
        Scanner scanner = new Scanner(hardwareMap, this);
        Drivetrain driveTrain = new Drivetrain(hardwareMap, this, 145.1, 1.0, 4.0);
        Collection collection = new Collection(hardwareMap, this);
        SpikeMarkDetection spikeMarkDetection = new SpikeMarkDetection(hardwareMap, this);
        int counter = 0;
        position = spikeMarkDetection.detectPosition(false);


        waitForStart();

//            This has a high value for a reason vv
        driveTrain.encoderDrive(.3, 24, 3);
        //score_on_spikemark
        driveTrain.turnToPID(90, 2);
        driveTrain.encoderDrive(.3, 45.5, 6);
        sleep(5000);
        driveTrain.strafeEncoderDrive(.3,12,2);
        driveTrain.encoderDrive(.3, 3, 3);
        //score_on_backdrop
        driveTrain.encoderDrive(.3, -2, 2);
        driveTrain.strafeEncoderDrive(.3,-24,3);
        driveTrain.encoderDrive(.3, 13, 2);

        sleep(5000);
        waitForStart();

        sleep(5000);


        /*
        //collection.moveClaw(.4, 3);
        if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {
            collection.rotatorServo.setPosition(.4);
            sleep(1000);
            driveTrain.encoderDrive(.7, 24, 3);
            driveTrain.turnToPID(90, 2);
            driveTrain.encoderDrive(.5, 6, 3);
            driveTrain.encoderDrive(.8, -10, 3);
            driveTrain.strafeEncoderDrive(.6, 28, 3);
            driveTrain.strafeEncoderDrive(.4, -5, 3);
            driveTrain.encoderDrive(.8, 86, 5);
            // lift.liftAuto(.7, 700, 4);
        }

        else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {
            collection.rotatorServo.setPosition(.4);
            sleep(1000);
            driveTrain.encoderDrive(.7, 30, 3);
            driveTrain.encoderDrive(.4, -12, 3);
            driveTrain.turnToPID(-89, 2);
            driveTrain.strafeEncoderDrive(.6, -22, 3);
            driveTrain.strafeEncoderDrive(.4, 5, 3);
            driveTrain.encoderDrive(.8, -90, 5);
        }
        else if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {
            collection.rotatorServo.setPosition(.4);
            sleep(1000);
            driveTrain.diagonalDriveRight(.7, -20, 3);
            driveTrain.encoderDrive(-.7, 15, 3);
            driveTrain.encoderDrive(.7, -20, 3);
            driveTrain.turnToPID(-89, 1.5);
            driveTrain.strafeEncoderDrive(.6, -12, 3);
            driveTrain.strafeEncoderDrive(.4, 5, 3);
            driveTrain.encoderDrive(.8, -90, 5);
        }

//        driveTrain.turnToPID(90, 2);
//        sleep(1000);
//        driveTrain.turnToPID(0, 2);
    */
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