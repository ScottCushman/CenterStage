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

        driveTrain.encoderDrive(.1,5,2);
        sleep(250);
        driveTrain.encoderDriveStart(.1,5,2);


        /*
        driveTrain.turnToPID(90,2);
        sleep(250);
        driveTrain.turnToPIDStart(90);

         */


        /*
        driveTrain.strafeEncoderDrive(.1,5,2);
        sleep(250);
        driveTrain.strafeEncoderDriveStart(.1,-5,2);

         */














    }
}