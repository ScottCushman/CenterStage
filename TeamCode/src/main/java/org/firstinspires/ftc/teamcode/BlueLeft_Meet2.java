
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import java.util.ArrayList;
@Disabled
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



           //base_code:


            waitForStart();
            /*

            driveTrain.encoderDrive(.3, 24, 3);
            driveTrain.turnToPID(90, 3);
            driveTrain.encoderDrive(.3, 36, 3);
            //lift_sliders
            //rotate_arm
            //box_release
            //box_close
            //rotate_arm
            //lower_sliders
            driveTrain.encoderDrive(.3, -3, 3);
            driveTrain.strafeEncoderDrive(.3, 24, 3);
            driveTrain.strafeEncoderDrive(.3, 13, 3);
             */

            Intake.spin(1,1);


            if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {
                //left_detection
                driveTrain.encoderDrive(.3, 24, 3);
                driveTrain.strafeEncoderDrive(.3, 6, 2);
                driveTrain.encoderDrive(.3, 1.5, 2);
                driveTrain.strafeEncoderDrive(.3, 12, 3);
                driveTrain.encoderDrive(.3, 1.5, 2);
                driveTrain.turnToPID(90, 3);
                driveTrain.encoderDrive(.3, 18, 3);
                //lift_sliders vv
                //Lift.something
                //rotate_arm to align with backdrop vv
                Collection.rotateArm(.88,2);
                //release the pixels* vv    *the setPosition values could be swapped i'm not sure
                Collection.rotateClawStart(0,2);
                //box_close vv
                Collection.rotateClawStart(0.2,2);
                //rotate_arm back down to align with bot vv
                Collection.rotateArm(.18,2);
                //lower_sliders vv
                //Lift.something
                driveTrain.encoderDrive(.3, -3, 3);
                driveTrain.strafeEncoderDrive(.3, -24, 3);
                driveTrain.strafeEncoderDrive(.3, 13, 3);

            }

            else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {


                //middle_detection
                driveTrain.encoderDrive(.3, 24, 3);
                driveTrain.encoderDrive(.3, -3, 2);
                driveTrain.strafeEncoderDrive(.3, 12, 2);
                driveTrain.encoderDrive(.3, 3, 2);
                driveTrain.turnToPID(90, 3);
                driveTrain.encoderDrive(.3, 24, 3);
                //move_sliders
                //rotate_arm
                //box_release
                //box_close
                //rotate_arm
                //move_sliders
                driveTrain.encoderDrive(.3, -3, 3);
                driveTrain.strafeEncoderDrive(.3, -24, 3);
                driveTrain.strafeEncoderDrive(.3, 13, 3);

            }

            else if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {


                //right_detection
                driveTrain.encoderDrive(.3, 24, 3);
                driveTrain.strafeEncoderDrive(.3, -6, 2);
                driveTrain.encoderDrive(.3, -1.5, 2);
                driveTrain.strafeEncoderDrive(.3, 12, 2);
                driveTrain.encoderDrive(.3, 1.5, 2);
                driveTrain.turnToPID(90, 3);
                driveTrain.encoderDrive(.3, 30, 3);
                //move_sliders
                //rotate_arm
                //box_release
                //box_close
                //rotate_arm
                //move_sliders
                driveTrain.encoderDrive(.3, -3, 3);
                driveTrain.strafeEncoderDrive(.3, -24, 3);
                driveTrain.strafeEncoderDrive(.3, 13, 3);
            }


























        }





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

