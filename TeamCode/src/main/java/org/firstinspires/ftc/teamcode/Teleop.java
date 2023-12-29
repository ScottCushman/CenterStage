package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.checkerframework.checker.units.qual.C;

import java.util.ArrayList;

//https:www.youtube.com/watch?v=pQ_aVTM9qX0
@TeleOp(name = "Trash Worst Robot In existence we should quit", group = "Iterative Opmode")
public class Teleop extends OpMode {
    Drivetrain drivetrain;
    Lift lift;
    Collection collection;
    ArrayList<Double> liftHeights = new ArrayList<Double>();
    Test test;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, this, 537.6, 1.0, 4.0);
        lift = new Lift(hardwareMap, this, 537.6, 1, 2, liftHeights);
        test = new Test(hardwareMap, this);
        liftHeights.add(1.0);
        liftHeights.add(7.0);
        liftHeights.add(15.0);
        liftHeights.add(16.1);
        liftHeights.add(1.8);
        liftHeights.add(16.1);
         collection = new Collection(hardwareMap, this);

        //  armAndClaw = new Arm_and_Claw(hardwareMap, this);

    }
    @Override
    public void loop() {
        drivetrain.UpdateDriveTrain();
      //  lift.raiseLiftToDesignatedPositionTeleop();
       drivetrain.DriverControls();
        //   rubberBandSpinner.rotateSpinnersTeleop();
      //  drivetrain.testDrivetrain();
        collection.collectionTeleop();
        collection.setCollectionPosition();
        lift.teleLift();
//        if (gamepad2.a) {
//            collection.imAboutToDie.setPosition(.5);
//        }
        collection.hangMotor.setPower(gamepad2.left_stick_y);

    }

    @Override
    public void stop() {
    }

}