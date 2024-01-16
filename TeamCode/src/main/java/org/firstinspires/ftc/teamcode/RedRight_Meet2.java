package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
@Disabled
@Autonomous
public class RedRight_Meet2 extends LinearOpMode {

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
        //Scanner scanner = new Scanner(hardwareMap, this);
        Drivetrain driveTrain = new Drivetrain(hardwareMap, this, 145.1, 1.0, 4.0);
        Collection collection = new Collection(hardwareMap, this);
        //SpikeMarkDetection spikeMarkDetection = new SpikeMarkDetection(hardwareMap, this);
        int counter = 0;
        //position = spikeMarkDetection.detectPosition(true);



        waitForStart();


        driveTrain.encoderDrive(.3, 24, 3);
        driveTrain.turnToPID(270, 3);
        driveTrain.encoderDrive(.3, 36, 3);
        //move_sliders
        //rotate_arm
        //box_release
        //box_close
        //rotate_arm
        //move_sliders
        driveTrain.encoderDrive(.3, -3, 3);
        driveTrain.strafeEncoderDrive(.3, -24, 3);
        driveTrain.strafeEncoderDrive(.3, 13, 3);




        //left_detection
        driveTrain.encoderDrive(.3, 24, 3);
        driveTrain.strafeEncoderDrive(.3, 6, 2);
        driveTrain.encoderDrive(.3, -1.5,2);
        driveTrain.strafeEncoderDrive(.3,-12,2);
        driveTrain.encoderDrive(.3,1.5,2);
        driveTrain.turnToPID(270, 3);
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

        //middle_detection
        driveTrain.encoderDrive(.3, 24, 3);
        driveTrain.encoderDrive(.3,-3,2);
        driveTrain.strafeEncoderDrive(.3,-12,2);
        driveTrain.encoderDrive(.3,3,2);
        driveTrain.turnToPID(270, 3);
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


        //right_detection
        driveTrain.encoderDrive(.3, 24, 3);
        driveTrain.strafeEncoderDrive(.3, -6, 2);
        driveTrain.encoderDrive(.3,1.5,2);
        driveTrain.strafeEncoderDrive(.3,-12,3);
        driveTrain.encoderDrive(.3,1.5,2);
        driveTrain.turnToPID(270, 3);
        driveTrain.encoderDrive(.3, 18, 3);
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