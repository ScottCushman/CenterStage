package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous
public class R1Auto extends LinearOpMode {

    private int codePosition;
    private int markPosition;

    public void runOpMode() {
        String tempString = "";
        ArrayList<Double> liftHeights = new ArrayList<Double>();
        liftHeights.add(2.2);
        liftHeights.add(3.6);
        liftHeights.add(16.4);
        liftHeights.add(5.8);
        liftHeights.add(26.5);
        liftHeights.add(0.5);
        liftHeights.add(33.0);
        //  Lift lift = new Lift(hardwareMap, this, liftHeights);
        Scanner scanner = new Scanner(hardwareMap, this);
       // Drivetrain driveTrain = new Drivetrain(hardwareMap, this, 537.6, 1.0, 4.0);
        SpikeMarkDetection spikeMarkDetection = new SpikeMarkDetection(hardwareMap, this);
        int counter = 0;
        markPosition = spikeMarkDetection.detectPosition();
        telemetry.addData("Current Pos", tempString);
        telemetry.addData(tempString, markPosition);
        telemetry.update();
        waitForStart();

//        driveTrain.turnToPID(90, 2);
//        sleep(1000);
//        driveTrain.turnToPID(0, 2);

        codePosition = scanner.scanSignal();
        telemetry.addData("Current Signal Zone", tempString);
        telemetry.addData(tempString, codePosition);
        telemetry.update();
    }




}