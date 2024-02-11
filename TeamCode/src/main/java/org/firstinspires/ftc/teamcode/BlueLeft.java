
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import java.util.ArrayList;
    @Autonomous
    public class BlueLeft extends LinearOpMode {

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
            Intake intake = new Intake(hardwareMap);
            Scanner scanner = new Scanner(hardwareMap, this);
            Drivetrain driveTrain = new Drivetrain(hardwareMap, this, 145.1, 1.0, 4.0);
            Collection collection = new Collection(hardwareMap, this);
            SpikeMarkDetection spikeMarkDetection = new SpikeMarkDetection(hardwareMap, this);
            int counter = 0;
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

                driveTrain.encoderDrive(.3, 24, 3);
                driveTrain.strafeEncoderDrive(.3, 6, 2);
                driveTrain.encoderDrive(.3, 1.5, 2);
                driveTrain.strafeEncoderDrive(.3, 12, 3);
                driveTrain.encoderDrive(.3, 1.5, 2);
                driveTrain.turnToPID(90, 3);
                driveTrain.encoderDrive(.3, 18, 3);
                //lift_sliders vv
                lift.liftAutoStart(0.2,1,1);
                //rotate_arm to align with backdrop vv
                collection.rotateArm(.88,2);
                //release the pixels* vv    *the setPosition values could be swapped i'm not sure
                collection.rotateClawStart(0,2);
                //box_close vv
                collection.rotateClawStart(0.2,2);
                //rotate_arm back down to align with bot vv
                collection.rotateArm(.18,2);
                //lower_sliders vv
                lift.liftAutoStart(0.2,0,1);
                driveTrain.encoderDrive(.3, -3, 3);
                driveTrain.strafeEncoderDrive(.3, -24, 3);
                driveTrain.strafeEncoderDrive(.3, 13, 3);




            if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {
                driveTrain.turnToPID(90, 2);
                sleep(2000);
                driveTrain.turnToPID(0, 2);

                //left_detection
//                driveTrain.encoderDrive(.3, 24, 3);
//                driveTrain.strafeEncoderDrive(.3, 6, 2);
//                driveTrain.encoderDrive(.3, 1.5, 2);
//                driveTrain.strafeEncoderDrive(.3, 12, 3);
//                driveTrain.encoderDrive(.3, 1.5, 2);
//                driveTrain.turnToPID(90, 3);
//                driveTrain.encoderDrive(.3, 18, 3);
//                //lift_sliders vv
//                lift.liftAutoStart(0.2,1,1);
//                //rotate_arm to align with backdrop vv
//                collection.rotateArm(.88,2);
//                //release the pixels* vv    *the setPosition values could be swapped i'm not sure
//                collection.rotateClawStart(0,2);
//                //box_close vv
//                collection.rotateClawStart(0.2,2);
//                //rotate_arm back down to align with bot vv
//                collection.rotateArm(.18,2);
//                //lower_sliders vv
//                lift.liftAutoStart(0.2,0,1);
//                driveTrain.encoderDrive(.3, -3, 3);
//                driveTrain.strafeEncoderDrive(.3, -24, 3);
//                driveTrain.strafeEncoderDrive(.3, 13, 3);

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






    }

