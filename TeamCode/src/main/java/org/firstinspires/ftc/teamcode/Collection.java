package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

public class Collection {
    private ElapsedTime runtime = new ElapsedTime();
    Servo rotatorServo;
    Servo rotServo;
    Servo imAboutToDie;
    Servo imGoingToDie;
    DcMotor hangMotor;
    DcMotor collectionMotor;
    NormalizedColorSensor colorSensor;
    private OpMode theOpMode;
    double armSpeed = 0;
    double imGoingToDie1;
    static final double imGoingToDie2 = .2;

    double imAboutToDie1;
    static final double imAboutToDie2 = .2;

    public Collection(HardwareMap hardwareMap, OpMode opMode) {
        theOpMode = opMode;
        rotatorServo = hardwareMap.get(Servo.class, "rotatorServo");
        rotServo = hardwareMap.get(Servo.class, "rotServo");
        imAboutToDie = hardwareMap.get(Servo.class, "leftBox");
        imGoingToDie = hardwareMap.get(Servo.class, "rightBox");
        hangMotor = hardwareMap.get(DcMotor.class, "hang");
        collectionMotor = hardwareMap.get(DcMotor.class, "collectionMotor");
        collectionMotor.setDirection(DcMotor.Direction.FORWARD);

        rotatorServo.setDirection(Servo.Direction.FORWARD);
        imGoingToDie.setDirection(Servo.Direction.REVERSE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imGoingToDie1 = imGoingToDie2;
        imAboutToDie1 = imAboutToDie2;
        //   colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
    }

    public void collectionTeleop() {
        /*
        if(theOpMode.gamepad2.y &&!theOpMode.gamepad2.x) {
            collectionMotor.setPower(-.4);
            collectionMotor.setTargetPosition(100);
            collectionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        else if(!theOpMode.gamepad2.y && theOpMode.gamepad2.x)

        {
            collectionMotor.setPower(.4);
            collectionMotor.setTargetPosition(5);
        }
        else {
            collectionMotor.setTargetPosition(collectionMotor.getCurrentPosition());
        }

         */

    }

    public void setCollectionPosition() {
       // collectionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        if (theOpMode.gamepad1.left_bumper || theOpMode.gamepad2.left_bumper) {
//            rotatorServo.setPosition(0);
//        } else if (theOpMode.gamepad1.right_bumper || theOpMode.gamepad2.right_bumper) {
//            rotatorServo.setPosition(.2);
//        }

        if (theOpMode.gamepad2.dpad_left || theOpMode.gamepad1. dpad_left) {
            rotServo.setPosition(.3);
        } else if (theOpMode.gamepad2.dpad_right || theOpMode.gamepad1.dpad_right) {
            rotServo.setPosition(0.6);
        }
        if (theOpMode.gamepad2.dpad_up || theOpMode.gamepad1.dpad_up) {
            imAboutToDie.setPosition(.8);
            imGoingToDie.setPosition(.8);
        } else if (theOpMode.gamepad2.dpad_down || theOpMode.gamepad1.dpad_down) {
            imAboutToDie.setPosition(.55);
            imGoingToDie.setPosition(.55);
        }
        collectionMotor.setPower(-theOpMode.gamepad2.right_stick_y * .5);
        if (theOpMode.gamepad1.x) {
            imAboutToDie.setPosition(.74);
            imGoingToDie.setPosition(.74);
        }

         if (theOpMode.gamepad1.right_bumper) {
            hangMotor.setPower(-1);
        }
         else if (theOpMode.gamepad1.left_bumper) {
             hangMotor.setPower(1);
         }
         else {
             hangMotor.setPower(theOpMode.gamepad2.left_stick_y);
         }




        // if (theOpMode.gamepad2.y) {
            /*
           rotatorServo.setPosition(.5);
            collectionMotor.setPower(.7);
            collectionMotor.setTargetPosition(150);
            collectionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotatorServo.setPosition(.8);
            collectionMotor.setPower(-.7);
            collectionMotor.setTargetPosition(5);

             */

          }
//        else {
//            collectionMotor.setTargetPosition(collectionMotor.getCurrentPosition());
//
//        }
//        if (theOpMode.gamepad2.x) {
//            collectionMotor.setTargetPosition(-150);
//            collectionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            collectionMotor.setPower(.4);
//        }






//}


    public void moveClaw(double clawPos, double timeoutS) {
        runtime.reset();
        while (((LinearOpMode) theOpMode).opModeIsActive() && (runtime.seconds() < timeoutS)) {
            rotatorServo.setPosition(clawPos);
        }

    }
    public void moveClawStart(double clawPos, double timeoutS) {
    }
    public boolean moveClawCheck(double clawPos, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() && (runtime.seconds() < timeoutS)) {
            rotatorServo.setPosition(clawPos);
            return true;
        }
        else {
            moveClawEnd();
            return false;
        }
    }
    public void moveClawEnd() {

    }

    public void rotateArm(double armPos, double timeoutS) {
        imAboutToDie.setPosition(armPos);
        imGoingToDie.setPosition(armPos);

    }

    public void rotateArmStart(double armPos, double timeoutS) {
        imGoingToDie.setPosition(armPos);
        imAboutToDie.setPosition(armPos);
    }

    public boolean rotateArmCheck(double armPos, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS) {
            imGoingToDie.setPosition(armPos);
            imAboutToDie.setPosition(armPos);
            return true;
        }
        rotateArmEnd();
        return false;
    }

    public void rotateArmEnd() {
    }

    public void rotateClaw(double position, double timeoutS) {
        while (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS) {
            rotServo.setPosition(position);
        }
    }

    public void rotateClawStart(double position, double timeoutS) {
        rotServo.setPosition(position);
    }

    public boolean rotateClawCheck(double position, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS) {
            rotServo.setPosition(position);
            return true;
        }
        rotateClawEnd();
        return false;
    }

    public void rotateClawEnd() {
    }

    public void collectionArm(int rotation, double power, double timeoutS) {
        runtime.reset();
        collectionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectionMotor.setTargetPosition(rotation);
        collectionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        collectionMotor.setPower(Math.abs(power));

        while (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS && collectionMotor.isBusy()) {
            theOpMode.telemetry.addData("targetPosition", 0);
            theOpMode.telemetry.addData("CurrentPosition", 0);

        }
        collectionMotor.setPower(0);
        collectionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void collectionArmStart(int rotation, double power, double timeoutS) {
        collectionMotor.setPower(power);
        collectionMotor.setTargetPosition(rotation);
        collectionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //something or rather
    }

    public boolean collectionArmCheck(int rotation, double power, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS && collectionMotor.isBusy()) {
        return true;
        }
        collectionArmEnd();
        return false;
    }
    public void collectionArmEnd() {
        collectionMotor.setPower(0);
    }
}

