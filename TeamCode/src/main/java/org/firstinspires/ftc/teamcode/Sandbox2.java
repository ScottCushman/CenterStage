package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import java.util.ArrayList;
@Autonomous
public class Sandbox2 extends LinearOpMode {

    private int codePosition;
    private SpikeMarkDetection.spikeMarkPositions position;

    public void runOpMode() {
        //  String tempString = "";
        //ArrayList<Double> liftHeights = new ArrayList<>();
        //liftHeights.add(3.0);
        //liftHeights.add(16.5);
        //liftHeights.add(3.0);
        //liftHeights.add(16.5);
        //Lift lift = new Lift(hardwareMap, this, 145.1, 1, 2, liftHeights);
        //Intake intake = new Intake(hardwareMap);
        //Scanner scanner = new Scanner(hardwareMap, this);
        Drivetrain driveTrain = new Drivetrain(hardwareMap, this, 145.1, 1.0, 4.0);
        //Collection collection = new Collection(hardwareMap, this);
        //SpikeMarkDetection spikeMarkDetection = new SpikeMarkDetection(hardwareMap, this);
        // int counter = 0;
        //position = spikeMarkDetection.detectPosition(false);

        waitForStart();
        //BlueLeft_Meet2
        //right_detection
        driveTrain.encoderDrive(.1, -15, 3);
        driveTrain.turnToPID(45,2);
        driveTrain.encoderDrive(.1, -7, 2);
        driveTrain.encoderDrive(.1,15,2);
        driveTrain.turnToPID(0,2);
        driveTrain.encoderDrive(.1,-10,2);
        driveTrain.turnToPID(90,2);
        sleep(250);
        driveTrain.encoderDrive(.5,23,3);
        //insert deliver
        sleep(2000);
        driveTrain.encoderDrive(.2,-7,2);
        sleep(500);
        driveTrain.turnToPID(178,2);
        driveTrain.encoderDrive(.1,-15,3);
        driveTrain.turnToPID(230,2);
        driveTrain.encoderDrive(.1,-6,2);
        driveTrain.turnToPID(250,2);
        driveTrain.encoderDrive(.1,-6,2);




        //make sure the surgical tubing is positioned so that all tubing is overshadowing the pixel and will not catch;
        //make sure the pixel pick-up sheet is tucked under the robot

        waitForStart();
        //BlueLeft_Meet2
        //middle_detection
        driveTrain.encoderDrive(.1, -26.5, 3);
        driveTrain.encoderDrive(.1,8,2);
        driveTrain.strafeEncoderDrive(.7,-15,2);
        driveTrain.turnToPID(-265, 4);
        sleep(250);
        driveTrain.encoderDrive(.5,26,3);
        sleep(500);
        driveTrain.turnToPID(-270,1);
        //insert delivery
        sleep(2000);
        driveTrain.encoderDrive(.2,-7,2);
        sleep(500);
        driveTrain.turnToPID(178,4);
        driveTrain.encoderDrive(.1,-10,3);
        driveTrain.turnToPID(245,4);
        driveTrain.encoderDrive(.1,-6,2);
        driveTrain.turnToPID(250,4);
        sleep(234567);





        waitForStart();
        //BlueLeft_Meet2
        //left_detection
        driveTrain.encoderDrive(.1, -15, 3);
        driveTrain.turnToPID(-45,3);
        driveTrain.encoderDrive(.1, -1.5, 2);
        driveTrain.encoderDrive(.1,12,2);
        driveTrain.turnToPID(-90,3);
        driveTrain.encoderDrive(.1,-15,2);
        driveTrain.turnToPID(0,3);
        driveTrain.strafeEncoderDrive(.7,-15,1);
        driveTrain.turnToPID(65,3);
        sleep(23456);
        //insert deliver
        sleep(2000);
        driveTrain.encoderDrive(.2,-7,2);
        sleep(500);
        driveTrain.turnToPID(178,2);
        driveTrain.encoderDrive(.1,-15,3);
        driveTrain.turnToPID(230,2);
        driveTrain.encoderDrive(.1,-6,2);
        driveTrain.turnToPID(250,2);
        driveTrain.encoderDrive(.1,-6,2);

    }
}