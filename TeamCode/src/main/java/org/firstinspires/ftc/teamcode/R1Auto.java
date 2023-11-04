package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous
public class R1Auto extends LinearOpMode {

    private int codePosition;
    public void runOpMode() {
        String tempString = "";
        ArrayList<Double> liftHeights = new ArrayList<Double>();
            liftHeights.add(3.0);
            liftHeights.add(16.5);
        liftHeights.add(3.0);
        liftHeights.add(16.5);
        //  Lift lift = new Lift(hardwareMap, this, 537.5,1, 2, liftHeights);
        Scanner scanner = new Scanner(hardwareMap, this);
       // Drivetrain driveTrain = new Drivetrain(hardwareMap, this, 537.6, 1.0, 4.0);
        SpikeMarkDetection spikeMarkDetection = new SpikeMarkDetection(hardwareMap, this);
        int counter = 0;
            telemetry.addData("Hi", tempString);
            telemetry.update();
           spikeMarkDetection.detectPosition();
            telemetry.addData("Current Pos", tempString);
            telemetry.addData(tempString, spikeMarkDetection.detectPosition());
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