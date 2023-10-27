package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.checkerframework.checker.units.qual.C;

import java.util.ArrayList;

//https:www.youtube.com/watch?v=pQ_aVTM9qX0
@TeleOp(name = "Trash Worst Robot In existence we should never build anything ever again", group = "Iterative Opmode")
public class Teleop extends OpMode {
    Drivetrain drivetrain;
    Lift lift;
    Collection collection;
    ArrayList<Double> liftHeights = new ArrayList<Double>();

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, this, 537.6, 1.0, 4.0);
        liftHeights.add(1.8);
        liftHeights.add(16.1);
        liftHeights.add(27.6);
        liftHeights.add(38.0);
        liftHeights.add(8.0);
        liftHeights.add(2.4);
      //  lift = new Lift(hardwareMap, this, 537.6, 1, 2);
         collection = new Collection(hardwareMap, this);
        //  armAndClaw = new Arm_and_Claw(hardwareMap, this);

    }

    @Override
    public void loop() {
        drivetrain.UpdateDriveTrain();
      //  lift.raiseLiftToDesignatedPositionTeleop();
        drivetrain.DriverControls();
        //   rubberBandSpinner.rotateSpinnersTeleop();
        collection.collectionTeleop();
        collection.setCollectionPosition();
        if (gamepad2.a) {
            collection.imAboutToDie.setPosition(.5);
            lift.teleLift();
        }
    }

    @Override
    public void stop() {
    }

}