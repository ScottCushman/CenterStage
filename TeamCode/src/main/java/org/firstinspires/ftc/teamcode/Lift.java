package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.ArrayList;

public class Lift {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DistanceSensor sensorRange;
    private ElapsedTime runtime = new ElapsedTime();
    private OpMode theOpMode;
    private ArrayList<Double> liftHeights;
    private int teleopLiftHeight = 0;
    private double targetHeight = 0;
    double countsPerInch;
    private double kp = 0.1;
    private double ki = 0.01;
    private double kd = 0.001;

    // Variables for PID control
    private double integral = 0;
    private double previousError = 0;



    public Lift(HardwareMap hardwareMap, OpMode opMode, double encoderTicksPerRev, double gearRatio, double wheelDiameter, ArrayList<Double> LiftHeights) {
        liftHeights = LiftHeights;
        countsPerInch = (encoderTicksPerRev * gearRatio) / (wheelDiameter * 3.14);
        theOpMode = opMode;
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
      //  rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
      //  rightMotor.setDirection(DcMotor.Direction.REVERSE);
     //   sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
      //  rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void teleLift() {
/*
        if (theOpMode.gamepad2.x) {
            leftMotor.setPower(-.2);
        }
        else {
            leftMotor.setPower(0);

            if (theOpMode.gamepad2.b) {
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftMotor.setPower(.7);
                leftMotor.setTargetPosition(400);
                if (leftMotor.getCurrentPosition() < 400) {
                    leftMotor.setPower(.7);
                }
                else {
                    leftMotor.setPower(0);
                    leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }
            /*
            if (theOpMode.gamepad2.a) {
               leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               leftMotor.setPower(-.3);
               leftMotor.setTargetPosition(5);
               if (leftMotor.getCurrentPosition() < 5) {
                   leftMotor.setPower(-.3);
               }
               else {
                   leftMotor.setPower(0);
                   leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               }

             */
           // }
    //        if (theOpMode.gamepad2.y) {
        leftMotor.setPower((theOpMode.gamepad2.left_stick_y) * .7);
       //     }

/*
            if (theOpMode.gamepad2.a) {
                int target;
                target = leftMotor.getCurrentPosition() + (int) (500 * countsPerInch);
                leftMotor.setTargetPosition(target);
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftMotor.setPower(Math.abs(.7));

                while (((LinearOpMode) theOpMode).opModeIsActive() && (leftMotor.isBusy())) {
                    leftMotor.setPower(Math.abs(.7));
                }
                leftMotor.setPower(0);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }



            // leftMotor.setPower(theOpMode.gamepad2.left_stick_y * .7);

/*
            targetHeight = leftMotor.getCurrentPosition() + (int) ((teleopLiftHeight * countsPerInch));
            leftMotor.setTargetPosition(((int) targetHeight));
            double rangeErrorOfLiftHeight = ((targetHeight) - leftMotor.getCurrentPosition());
            double speedOfLiftForDesignatedPosition = (rangeErrorOfLiftHeight / 3);
            speedOfLiftForDesignatedPosition = Math.max(-.2, Math.min(1, speedOfLiftForDesignatedPosition));
            leftMotor.setPower(speedOfLiftForDesignatedPosition);
            if (leftMotor.getCurrentPosition() > 400) {
                leftMotor.setPower(0);
            }

 */

        }
        /*
        if (Math.abs(theOpMode.gamepad2.right_stick_y) > .1) {
            targetHeight = leftMotor.getCurrentPosition() - (theOpMode.gamepad2.right_stick_y * 3);
        }
        if ((targetHeight) > (leftMotor.getCurrentPosition())) {
            double rangeErrorOfLiftHeight = (targetHeight) - leftMotor.getCurrentPosition();
            double speedOfLiftForDesignatedPosition = (rangeErrorOfLiftHeight / 10);
            speedOfLiftForDesignatedPosition = Math.max(-.2, Math.min(1, speedOfLiftForDesignatedPosition));
            leftMotor.setPower(speedOfLiftForDesignatedPosition);

            //theOpMode.telemetry.addData("range", String.format("%.2f", speedOfLiftForDesignatedPosition));
            //theOpMode.telemetry.update();

        } else {
            //Moving lift down
            //    theOpMode.telemetry.addData("Moving", "down");
            double rangeErrorOfLiftHeight = (targetHeight) - leftMotor.getCurrentPosition();
            double speedOfLiftForDesignatedPosition = (rangeErrorOfLiftHeight / 40);
            speedOfLiftForDesignatedPosition = Math.max(-1, Math.min(1, speedOfLiftForDesignatedPosition));
            leftMotor.setPower(speedOfLiftForDesignatedPosition);


        }

         */
  //  }
    public void liftAuto(double speed, double distance, double timeoutS) {
        int target;
        target = leftMotor.getCurrentPosition() + (int) (distance * countsPerInch);
        leftMotor.setTargetPosition(target);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(Math.abs(speed));

        while (((LinearOpMode) theOpMode).opModeIsActive() &&
                (runtime.seconds() < timeoutS) && (leftMotor.isBusy())) {
            leftMotor.setPower(Math.abs(speed));
        }
        leftMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void liftAutoStart(double speed, double distance, double timeoutS) {
        int target;
        target = leftMotor.getCurrentPosition() + (int) (distance * countsPerInch);
        leftMotor.setTargetPosition(target);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(Math.abs(speed));
    }
    public boolean liftAutoCheck(double speed, double distance, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() &&
                (runtime.seconds() < timeoutS) && (leftMotor.isBusy())) {
            leftMotor.setPower(Math.abs(speed));
            return true;
        }
        else {
            liftAutoEnd();
            return false;
        }
    }

    public void liftAutoEnd() {
        leftMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }















    public void raiseLiftToDesignatedPosition(int designatedPosition, int timeoutS) {

        runtime.reset();
        while (((LinearOpMode) theOpMode).opModeIsActive() &&
                Math.abs(liftHeights.get(designatedPosition) - (sensorRange.getDistance(DistanceUnit.INCH))) > .9 &&
                (runtime.seconds() < timeoutS)) {
            if ((liftHeights.get(designatedPosition)) > (sensorRange.getDistance(DistanceUnit.INCH))) {
                double rangeErrorOfLiftHeight = liftHeights.get(designatedPosition) - sensorRange.getDistance(DistanceUnit.INCH);
                double speedOfLiftForDesignatedPosition = (rangeErrorOfLiftHeight / 3);
                leftMotor.setPower(speedOfLiftForDesignatedPosition);
                rightMotor.setPower(speedOfLiftForDesignatedPosition);

            } else {

                double rangeErrorOfLiftHeight = liftHeights.get(designatedPosition) - sensorRange.getDistance(DistanceUnit.INCH);
                double speedOfLiftForDesignatedPosition = (rangeErrorOfLiftHeight / 3);
                rightMotor.setPower(speedOfLiftForDesignatedPosition);
                leftMotor.setPower(speedOfLiftForDesignatedPosition);
            }
        }
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

    public void raiseLiftToDesignatedPositionStart(int designatedPosition, double timeoutS) {
        runtime.reset();
        double rangeErrorOfLiftHeight = liftHeights.get(designatedPosition) - sensorRange.getDistance(DistanceUnit.INCH);
        double speedOfLiftForDesignatedPosition = (rangeErrorOfLiftHeight / 3);
        leftMotor.setPower(speedOfLiftForDesignatedPosition);
        rightMotor.setPower(speedOfLiftForDesignatedPosition);

    }

    public boolean raiseLiftToDesignatedPositionCheck(int designatedPosition, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() &&
                Math.abs(liftHeights.get(designatedPosition) - (sensorRange.getDistance(DistanceUnit.INCH))) > 1.5 &&
                (runtime.seconds() < timeoutS)) {
            double rangeErrorOfLiftHeight = liftHeights.get(designatedPosition) - sensorRange.getDistance(DistanceUnit.INCH);
            double speedOfLiftForDesignatedPosition = (rangeErrorOfLiftHeight / 3);
            leftMotor.setPower(speedOfLiftForDesignatedPosition);
            rightMotor.setPower(speedOfLiftForDesignatedPosition);
            theOpMode.telemetry.addData("Current Position", sensorRange.getDistance(DistanceUnit.INCH));
            theOpMode.telemetry.update();
            return true;
        } else {
            raiseLiftToDesignatedPositionEnd();
            return false;
        }
    }

    public void raiseLiftToDesignatedPositionTest(int designatedPosition, double timeoutS) {
        raiseLiftToDesignatedPositionStart(designatedPosition, timeoutS);
        while (raiseLiftToDesignatedPositionCheck(designatedPosition, timeoutS)) ;
    }

    public void raiseLiftToDesignatedPositionEnd() {
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

    public void raiseLiftToDesignatedPositionTeleop() {
        theOpMode.telemetry.addData("Running to", " %.01f", targetHeight);
        theOpMode.telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
        theOpMode.telemetry.update();
        if (theOpMode.gamepad2.b) {
            teleopLiftHeight = 3;
            targetHeight = liftHeights.get(teleopLiftHeight);
        }
        if (theOpMode.gamepad2.a) {
            teleopLiftHeight = 0;
            targetHeight = liftHeights.get(teleopLiftHeight);
        }
        if (theOpMode.gamepad2.x) {
            teleopLiftHeight = 1;
            targetHeight = liftHeights.get(teleopLiftHeight);
        }
        if (theOpMode.gamepad2.y) {
            teleopLiftHeight = 2;
            targetHeight = liftHeights.get(teleopLiftHeight);

        }
        if (theOpMode.gamepad2.dpad_up) {
            teleopLiftHeight = 4;
            targetHeight = liftHeights.get(teleopLiftHeight);

        }
        if (theOpMode.gamepad2.dpad_down) {
            teleopLiftHeight = 5;
            targetHeight =
                    liftHeights.get(teleopLiftHeight);
        }

        if (Math.abs(theOpMode.gamepad2.right_stick_y) > .1) {
            targetHeight = sensorRange.getDistance(DistanceUnit.INCH) - (theOpMode.gamepad2.right_stick_y * 3);
            theOpMode.telemetry.addData("Running to", " %.01f", targetHeight);
            theOpMode.telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
            theOpMode.telemetry.update();
        }

        // Moving lift up
//        if ((targetHeight) - (sensorRange.getDistance(DistanceUnit.INCH)) > 1.2 ||
//                ((targetHeight) - (sensorRange.getDistance(DistanceUnit.INCH)) < -1.2)) {
//            theOpMode.telemetry.addData("Moving", "up");
//            theOpMode.telemetry.update();
        if ((targetHeight) > (sensorRange.getDistance(DistanceUnit.INCH))) {
            double rangeErrorOfLiftHeight = (targetHeight) - sensorRange.getDistance(DistanceUnit.INCH);
            double speedOfLiftForDesignatedPosition = (rangeErrorOfLiftHeight / 6.69);
            speedOfLiftForDesignatedPosition = Math.max(-.2, Math.min(1, speedOfLiftForDesignatedPosition));
            rightMotor.setPower(speedOfLiftForDesignatedPosition);
            leftMotor.setPower(speedOfLiftForDesignatedPosition);

            //theOpMode.telemetry.addData("range", String.format("%.2f", speedOfLiftForDesignatedPosition));
            //theOpMode.telemetry.update();

        } else {
            //Moving lift down
            //    theOpMode.telemetry.addData("Moving", "down");
            double rangeErrorOfLiftHeight = (targetHeight) - sensorRange.getDistance(DistanceUnit.INCH);
            double speedOfLiftForDesignatedPosition = (rangeErrorOfLiftHeight / 6.69);
            speedOfLiftForDesignatedPosition = Math.max(-1, Math.min(1, speedOfLiftForDesignatedPosition));
            /*
            theOpMode.telemetry.addData("Error", rangeErrorOfLiftHeight);
            theOpMode.telemetry.addData("Speed", speedOfLiftForDesignatedPosition);
            theOpMode.telemetry.addData("Pos", rightMotor.getCurrentPosition());
            theOpMode.telemetry.update();

             */

            // if (rightMotor.getCurrentPosition() > 0) {

            rightMotor.setPower(speedOfLiftForDesignatedPosition);
            leftMotor.setPower(speedOfLiftForDesignatedPosition);

//                } else {
//                    rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                    leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                }

        }

    }

    public final boolean opModeIsActive() {
        return false;
    }



    public void runOpMode () {

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;
        theOpMode.telemetry.addData(">>", "Press start to continue");
        theOpMode.telemetry.update();

        while (opModeIsActive()) {
            // generic DistanceSensor methods.
            theOpMode.telemetry.addData("deviceName", sensorRange.getDeviceName());
            theOpMode.telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
            // Rev2mDistanceSensor specific methods.
            theOpMode.telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            theOpMode.telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
            theOpMode.telemetry.update();
        }


    }
    // https://www.youtube.com/watch?v=HPk-VhRjNI8&list=PL3KnTfyhrIlcudeMemKd6rZFGDWyK23vx&index=1
}