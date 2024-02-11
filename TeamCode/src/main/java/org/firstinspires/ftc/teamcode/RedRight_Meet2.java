package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
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

        if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {

            //left_detection

        }

        else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {

            //middle_detection


        }

        else if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {


            //right_detection


        }


















    }
}