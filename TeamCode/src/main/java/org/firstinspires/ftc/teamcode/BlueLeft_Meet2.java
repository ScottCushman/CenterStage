
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

    @Autonomous
    public class BlueLeft_Meet2 extends LinearOpMode {

        private int codePosition;
        private SpikeMarkDetection.spikeMarkPositions position;

        public void runOpMode() {
            //  String tempString = "";
            ArrayList<Double> liftHeights = new ArrayList<>();
            liftHeights.add(3.0);
            liftHeights.add(16.5);
            liftHeights.add(3.0);
            liftHeights.add(16.5);
            Lift lift = new Lift(hardwareMap, this, 145.1, 1, 2, liftHeights);
            Scanner scanner = new Scanner(hardwareMap, this);
            Drivetrain driveTrain = new Drivetrain(hardwareMap, this, 145.1, 1.0, 4.0);
            Collection collection = new Collection(hardwareMap, this);
            SpikeMarkDetection spikeMarkDetection = new SpikeMarkDetection(hardwareMap, this);
            // int counter = 0;
            position = spikeMarkDetection.detectPosition(false);






            waitForStart();

            driveTrain.encoderDrive(.3, 12, 3);
            driveTrain.turnToPID(90, 2);
            driveTrain.encoderDrive(.3,18,3);
            //score_on_backdrop
            sleep(1000);
            driveTrain.encoderDrive(.3,-2,2);
            driveTrain.strafeEncoderDrive(.3,-24,3);
            driveTrain.encoderDrive(.3, 13, 2);






















            sleep(5000);
            waitForStart();


            //Left Detection
            driveTrain.encoderDrive(.3, 24, 3);
            driveTrain.strafeEncoderDrive(.3, -6, 2);
            driveTrain.encoderDrive(.3, -4, 2);
            driveTrain.strafeEncoderDrive(.3, 6, 2);
            driveTrain.encoderDrive(.3, 4, 2);
            driveTrain.turnToPID(90, 2);
            driveTrain.encoderDrive(.3,18,3);
            //score_on_backdrop
            driveTrain.strafeEncoderDrive(.3,-24,3);
            driveTrain.encoderDrive(.3, 24, 2);

            sleep(20000);

            //Middle Detection
            driveTrain.encoderDrive(.3, 24, 3);
            driveTrain.encoderDrive(.3, -4, 2);
            driveTrain.strafeEncoderDrive(.3, 6, 2);
            driveTrain.encoderDrive(.3, 4, 2);
            driveTrain.turnToPID(90, 2);
            driveTrain.encoderDrive(.3,12,3);
            //score_on_backdrop
            driveTrain.strafeEncoderDrive(.3,-24,3);
            driveTrain.encoderDrive(.3, 24, 2);

            sleep(20000);

            //Right Detection
            driveTrain.encoderDrive(.3, 24, 3);
            driveTrain.strafeEncoderDrive(.3, 6, 2);
            driveTrain.encoderDrive(.3, -4, 2);
            driveTrain.strafeEncoderDrive(.3, 6, 2);
            driveTrain.encoderDrive(.3, 4, 2);
            driveTrain.turnToPID(90, 2);
            driveTrain.encoderDrive(.3,12,3);
            //score_on_backdrop
            driveTrain.strafeEncoderDrive(.3,-24,3);
            driveTrain.encoderDrive(.3, 24, 2);

            //collection.moveClaw(.4, 3);
           /* if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {
                driveTrain.encoderDrive(.7, 24, 3);
                driveTrain.turnToPID(90, 2);
                driveTrain.encoderDrive(.5, 6, 3);
                driveTrain.encoderDrive(.8, -6, 3);
                driveTrain.strafeEncoderDrive(.7, -30, 3);

            }

            else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {
                collection.moveClaw(.42, 3);
                sleep(1000);
                driveTrain.encoderDrive(.7, 35, 3);
                driveTrain.encoderDrive(.4, -15, 3);
                driveTrain.turnToPID(90, 1.5);
                driveTrain.strafeEncoderDrive(.4, -38, 3);
            }
            else if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {
                collection.moveClaw(.42, 3);
                sleep(1000);
                driveTrain.diagonalDriveRight(.7, -20, 3);
                driveTrain.encoderDrive(-.7, 15, 3);
                driveTrain.encoderDrive(.7, -10, 3);
                driveTrain.diagonalDriveRight(.7, 20, 3);
                driveTrain.turnToPID(90, 1.5);
                driveTrain.strafeEncoderDrive(.7, -10, 3);
            }
            driveTrain.strafeEncoderDrive(.5, 4, 3);
            driveTrain.encoderDrive(.9, 55, 3);


            collection.rotatorServo.setPosition(.4);
            sleep(1000);
            rotateClaw(.8, .3, 3, collection);
            sleep(1000);
            lift.liftAuto(.7, 20, 3);
            collection.rotatorServo.setPosition(.7);
            scanner.scanSignal();
            if (codePosition == 1) {
                collection.rotatorServo.setPosition(.4);
                sleep(1000);
                rotateClaw(.8, .3, 3, collection);
                sleep(1000);
                lift.liftAuto(.7, 20, 3);
                collection.rotatorServo.setPosition(.7);
            }
            else {
                driveTrain.strafeEncoderDrive(.4, -8, 3);
            }
            scanner.scanSignal();
            if (codePosition == 2) {
                collection.rotatorServo.setPosition(.4);
                sleep(1000);
                rotateClaw(.8, .3, 3, collection);
                sleep(1000);
                lift.liftAuto(.7, 20, 3);
                collection.rotatorServo.setPosition(.7);
            }
            else {
                driveTrain.strafeEncoderDrive(.4, -8, 3);


            }*/
        }


//        driveTrain.turnToPID(90, 2);
//        sleep(1000);
//        driveTrain.turnToPID(0, 2);



        public void driveScan(double speed, double inches, double timeoutS, Drivetrain givenDriveTrain, Scanner givenScanner) {
            givenDriveTrain.strafeEncoderDriveStart(speed, inches, timeoutS);
            givenScanner.scanSignal();
            while(opModeIsActive() && givenDriveTrain.strafeEncoderDriveCheck(speed, inches, timeoutS)) {
                codePosition = givenScanner.scanSignal();
                if (codePosition >= 3) {
                    givenDriveTrain.encoderDrive(.6, 3, 3);
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

