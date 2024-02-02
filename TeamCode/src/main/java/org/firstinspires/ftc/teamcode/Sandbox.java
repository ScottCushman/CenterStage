package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import java.util.ArrayList;
@Autonomous
public class Sandbox extends LinearOpMode {

    private int codePosition;
    private SpikeMarkDetection.spikeMarkPositions position;

    public void runOpMode() throws InterruptedException {
        //  String tempString = "";
        //ArrayList<Double> liftHeights = new ArrayList<>();
        //liftHeights.add(3.0);
        //liftHeights.add(16.5);
        //liftHeights.add(3.0);
        //liftHeights.add(16.5);
        //Lift lift = new Lift(hardwareMap, this, 145.1, 1, 2, liftHeights);
        Intake intake = new Intake(hardwareMap);
        //Scanner scanner = new Scanner(hardwareMap, this);
        Drivetrain driveTrain = new Drivetrain(hardwareMap, this, 145.1, 1.0, 4.0);
        Collection collection = new Collection(hardwareMap, this);
        //SpikeMarkDetection spikeMarkDetection = new SpikeMarkDetection(hardwareMap, this);
        // int counter = 0;
        //position = spikeMarkDetection.detectPosition(false);

        waitForStart();

        intake.spin(2,true);
        intake.spin(5,false);



       /* waitForStart();
        //RedRight_Meet2
        //left_detection

        driveTrain.encoderDrive(.1, -15, 3);
        driveTrain.turnToPID(45,2);
        driveTrain.encoderDrive(.1, -7, 2);
        driveTrain.encoderDrive(.1,15,2);
        driveTrain.turnToPID(0,2);
        driveTrain.encoderDrive(.1,-10,2);
        driveTrain.turnToPID(90,2);
        sleep(250);
        driveTrain.encoderDrive(.2,22,3);
        //insert delivery
        sleep(2000);
        driveTrain.encoderDrive(.2,-7,2);
        driveTrain.turnToPID(178,2);
        driveTrain.encoderDrive(.1,-18,3);
        driveTrain.turnToPID(230,2);
        driveTrain.encoderDrive(.1,-6,3);
        driveTrain.turnToPID(250,2);
        driveTrain.encoderDrive(.1,-6,4);

        */




        //make sure the surgical tubing is positioned so that all tubing is overshadowing the pixel and will not catch;

        /*waitForStart();
        //RedRight_Meet2
        //middle_detection
        driveTrain.encoderDrive(.1, -26.5, 3);
        driveTrain.encoderDrive(.1,8,2);
        driveTrain.strafeEncoderDrive(.7,15,2);
        driveTrain.turnToPID(-255, 4);
        sleep(250);
        driveTrain.encoderDrive(.1,30,2);
        sleep(250);
        driveTrain.turnToPID(-270,1.5);
        driveTrain.encoderDrive(.1,4,1);
        //insert delivery
        sleep(2000);
        driveTrain.encoderDrive(.2,-5,2);
        driveTrain.turnToPID(178,2);
        driveTrain.encoderDrive(.1,-11.5,3);
        driveTrain.turnToPID(250,2);
        driveTrain.encoderDrive(.1,-10,3);
        driveTrain.turnToPID(250,2);*/





        /*waitForStart();
        //RedRight_Meet2
        //right_detection
        driveTrain.encoderDrive(.1, -17, 3);
        sleep(250);
        driveTrain.turnToPID(-45,3);
        driveTrain.encoderDrive(.1, -2.5, 2);
        driveTrain.encoderDrive(.1,14,2);
        driveTrain.turnToPID(-90,3);
        driveTrain.encoderDrive(.1,-20,2);
        driveTrain.turnToPID(0,3);
        driveTrain.strafeEncoderDrive(.7,9,1);
        driveTrain.turnToPID(90,3);
        driveTrain.encoderDrive(.1,17,3);
        driveTrain.turnToPID(90,2);
        driveTrain.encoderDrive(.1,4,1);
        //insert deliver
        sleep(2000);
        driveTrain.encoderDrive(.2,-7,2);
        sleep(500);
        driveTrain.turnToPID(178,2);
        driveTrain.encoderDrive(.1,-4,3);
        driveTrain.turnToPID(230,2);
        driveTrain.encoderDrive(.1,-6,2);
        driveTrain.turnToPID(250,2);
        driveTrain.encoderDrive(.1,-6,2);
         */




        //RedLeft_Meet2 begins here





        //RedLeft_Meet2
        //left_detection
        /*waitForStart();
        driveTrain.encoderDrive(.1, -15, 3);
        driveTrain.turnToPID(45,2);
        driveTrain.encoderDrive(.1, -2, 2);
        driveTrain.encoderDrive(.1,3.5,2);
        driveTrain.turnToPID(0,2);
        driveTrain.encoderDrive(.1,-22,2);
        driveTrain.turnToPID(90,2);
        sleep(250);
        driveTrain.encoderDrive(.3,50,5);
        sleep(500);
        driveTrain.turnToPID(180,3);
        driveTrain.encoderDrive(.1,-22,3);
        driveTrain.turnToPID(90,3);
        driveTrain.encoderDrive(.1,8,3);
        //insert delivery
         */

        //stuff below here has not been tested vv


        //RedLeft_Meet2
        //Middle_detection
        /*waitForStart();
        driveTrain.encoderDrive(.1, -26.5, 3);
        driveTrain.encoderDrive(.1,8,2);
        driveTrain.turnToPID(45,2);
        driveTrain.encoderDrive(.1,-10,2);
        driveTrain.turnToPID(0, 2);
        driveTrain.encoderDrive(.1,-10,2);
        driveTrain.turnToPID(90,2);
        sleep(250);
        driveTrain.encoderDrive(.3,50,5);
        sleep(500);
        driveTrain.turnToPID(180,3);
        driveTrain.encoderDrive(.1,-27,3);
        driveTrain.turnToPID(90,3);
        driveTrain.encoderDrive(.1,8,3);

         */
        //insert delivery



        //RedLeft_Meet2
        //right_detection
        /*driveTrain.encoderDrive(.1, -15, 3);
        driveTrain.turnToPID(-45,2);
        driveTrain.encoderDrive(.1, -2, 2);
        driveTrain.encoderDrive(.1,3.5,2);
        driveTrain.turnToPID(0,2);
        driveTrain.encoderDrive(.1,-22,2);
        driveTrain.turnToPID(90,2);
        sleep(250);
        driveTrain.encoderDrive(.3,50,5);
        sleep(500);
        driveTrain.turnToPID(180,3);
        driveTrain.encoderDrive(.1,-32,3);
        driveTrain.turnToPID(90,3);
        driveTrain.encoderDrive(.1,8,3);*/















    }
}