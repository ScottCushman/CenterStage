
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous
public class RedLeft_Meet2 extends LinearOpMode {

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
        position = spikeMarkDetection.detectPosition(true);

        waitForStart();
//            This has a high value for a reason vv
        driveTrain.encoderDrive(.3, 22, 3);
        //score_on_spikemark
        driveTrain.turnToPID(270, 2);
        driveTrain.encoderDrive(.3, 56, 6);
        //score_on_backdrop
        sleep(5000);
        driveTrain.strafeEncoderDrive(.3,-12,2);
        driveTrain.encoderDrive(.3, -12, 3);
        driveTrain.encoderDrive(.3, 24, 2);


        sleep(5000);



        //Left Detection
        driveTrain.encoderDrive(.7, 24, 3);
        driveTrain.strafeEncoderDrive(.4, 6, 2);
        driveTrain.encoderDrive(.7,-12, 2);
        driveTrain.strafeEncoderDrive(.7, -12, 2);
        driveTrain.encoderDrive(.7,48,4);
        driveTrain.turnToPID(270, 2);
        driveTrain.encoderDrive(1, 120, 6);
        driveTrain.strafeEncoderDrive(.7, 12, 3);
        driveTrain.encoderDrive(.7, 6, 2);
        //Rotate_arm 180°
        //Rotate_arm -180°
        driveTrain.strafeEncoderDrive(.7, -18, 3);
        driveTrain.encoderDrive(.7, 12, 2);


        //Middle Detection
        driveTrain.encoderDrive(.7, 24, 3);
        driveTrain.encoderDrive(.7,-12, 2);
        driveTrain.strafeEncoderDrive(.7, -12, 2);
        driveTrain.encoderDrive(.7,48,4);
        driveTrain.turnToPID(270, 2);
        driveTrain.encoderDrive(1, 126, 6);
        driveTrain.strafeEncoderDrive(.7, 12, 3);
        driveTrain.encoderDrive(.7, 6, 2);
        //Rotate_arm 180°
        //Rotate_arm -180°
        driveTrain.strafeEncoderDrive(.7, -18, 3);
        driveTrain.encoderDrive(.7, 12, 2);


        //RIght Detection
        driveTrain.encoderDrive(.7, 24, 3);
        driveTrain.strafeEncoderDrive(.4, -6, 2);
        driveTrain.encoderDrive(.7,-12, 2);
        driveTrain.strafeEncoderDrive(.7, -12, 2);
        driveTrain.encoderDrive(.7,48,4);
        driveTrain.turnToPID(270, 2);
        driveTrain.encoderDrive(1, 114, 6);
        driveTrain.strafeEncoderDrive(.7, 12, 3);
        driveTrain.encoderDrive(.7, 6, 2);
        //Rotate_arm 180°
        //Rotate_arm -180°
        driveTrain.strafeEncoderDrive(.7, -18, 3);
        driveTrain.encoderDrive(.7, 12, 2);







        /*
        //collection.moveClaw(.4, 3);
        if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {
            collection.rotatorServo.setPosition(.4);
            sleep(1000);
            driveTrain.diagonalDriveLeft(.7, -20, 3);
            driveTrain.encoderDrive(-.7, 15, 3);
            driveTrain.encoderDrive(.7, -10, 3);
            driveTrain.turnToPID(-89, 1.5);
            driveTrain.encoderDrive(.5, 7, 3);
            driveTrain.strafeEncoderDrive(.6, -25, 3);
            driveTrain.strafeEncoderDrive(.4, 5, 3);
            driveTrain.encoderDrive(.8, 90, 3);

            // armClawLift(.8, .1, .7, 700, 3, collection, lift);

            // lift.liftAuto(.7, 700, 4);
        }

        else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {
            collection.rotatorServo.setPosition(.4);
            sleep(1000);
            driveTrain.encoderDrive(.7, 30, 3);
            driveTrain.encoderDrive(.4, -10, 3);
            driveTrain.turnToPID(-89, 2);
            driveTrain.encoderDrive(.5, -5, 3);
            driveTrain.strafeEncoderDrive(.8, -30, 3);
            driveTrain.strafeEncoderDrive(.4, 5, 3);
            //Change this for alliance partners
            driveTrain.encoderDrive(.9, 85, 5);
            //  driveTrain.strafeEncoderDrive();

        }
        else if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {
            collection.rotatorServo.setPosition(.4);
            sleep(1000);
            driveTrain.encoderDrive(.7, 24, 3);
            driveTrain.turnToPID(-89, 2);
            driveTrain.encoderDrive(.5, -6, 3);
            driveTrain.encoderDrive(.5, 8, 3);
            driveTrain.strafeEncoderDrive(.6, -30, 4);
            driveTrain.strafeEncoderDrive(.4, 5, 3);
            driveTrain.encoderDrive(.8, 90, 3);

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

   */
    }
}