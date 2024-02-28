package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous
public class BlueRight extends LinearOpMode {

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
        Scanner scanner = new Scanner(hardwareMap, this);
        Drivetrain driveTrain = new Drivetrain(hardwareMap, this, 145.1, 1.0, 4.0);
        Collection collection = new Collection(hardwareMap, this);
        SpikeMarkDetection spikeMarkDetection = new SpikeMarkDetection(hardwareMap, this);
        int counter = 0;
        position = spikeMarkDetection.detectPosition(false);








        waitForStart();
        //right detection
        driveTrain.awDrive(.15, .45, .15, .45, -15, 2);
        sleep(500);
        driveTrain.encoderDrive(.1,11.5,2);
        driveTrain.turnToPID(0,2);
        driveTrain.encoderDrive(.2,-26.5,2);
        driveTrain.turnToPID(90,2);
        driveTrain.encoderDrive(.25,-45,2);
        driveTrain.turnToPID(0,2);
        driveTrain.encoderDrive(.1,22,2);
        driveTrain.turnToPID(270,2);
        driveTrain.encoderDrive(.1,0.1,2);
        driveTrain.driveToDistanceSensor(.2,2,4);
        liftBox(.8, 500, .84, 2, lift, collection);
        collection.rotServo.setPosition(0);
        liftBox(-.3, -300, .55, 2, lift, collection);
        driveTrain.encoderDrive(.1,-4,2);
        sleep(250);
        liftBox(-.3, -300, .55, 2, lift, collection);




        if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {



        }

        else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {


        }

        else if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {
            driveTrain.awDrive(.45, .15, .45, .15, -15, 2);
            sleep(500);
            driveTrain.encoderDrive(.1,11.5,2);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.2,-26.5,2);
            driveTrain.turnToPID(90,2);
            driveTrain.encoderDrive(.25,-45,2);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,14,2);
            driveTrain.turnToPID(270,2);
            driveTrain.encoderDrive(.1,0.1,2);
            driveTrain.driveToDistanceSensor(.2,2,4);
            liftBox(.8, 500, .84, 2, lift, collection);
            collection.rotServo.setPosition(0);
            liftBox(-.3, -300, .55, 2, lift, collection);
            driveTrain.encoderDrive(.1,-4,2);
            sleep(250);
            liftBox(-.3, -300, .55, 2, lift, collection);



            }
















    }

    public void liftBox(double power, int target, double armPos, double timeoutS, Lift givenLift, Collection givenCollection) {
        givenLift.liftAutoStart(power, target, timeoutS);
        givenCollection.rotateArmStart(armPos, timeoutS);
        while (opModeIsActive() && givenLift.liftAutoCheck(power, target, timeoutS) || givenCollection.rotateArmCheck(armPos, timeoutS));
    }
    public void driveRotate(double speed, double inches, double clawPos, double timeoutS, Drivetrain givenDrivetrain, Collection givenCollection) {
        givenDrivetrain.encoderDriveStart(speed, inches, timeoutS);
        givenCollection.moveClawStart(clawPos, timeoutS);
        while (opModeIsActive() && givenDrivetrain.encoderDriveCheck(speed, inches, timeoutS) || givenCollection.moveClawCheck(clawPos, timeoutS));
    }
    public void driveRotateSpin(double speed, double inches, double clawPos, double power, double rotations, double timeoutS, Drivetrain givenDrivetrain, Collection givenCollection) {
        givenDrivetrain.encoderDriveStart(speed, inches, timeoutS);
        givenCollection.moveClawStart(clawPos, timeoutS);
        givenCollection.collectionStart(power, rotations, timeoutS);
        while (opModeIsActive() && givenDrivetrain.encoderDriveCheck(speed, inches, timeoutS) || givenCollection.moveClawCheck(clawPos, timeoutS) || givenCollection.collectionCheck(power, rotations, timeoutS));
    }
}
