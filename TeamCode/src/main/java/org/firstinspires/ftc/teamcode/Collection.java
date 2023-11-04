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
    NormalizedColorSensor colorSensor;
    private OpMode theOpMode;
    double armSpeed = 0;

    public Collection(HardwareMap hardwareMap, OpMode opMode) {
        theOpMode = opMode;
        rotatorServo = hardwareMap.get(Servo.class, "rotatorServo");
        rotServo = hardwareMap.get(Servo.class, "rotServo");
        imAboutToDie = hardwareMap.get(Servo.class, "armServo");

        rotatorServo.setDirection(Servo.Direction.FORWARD);
     //   colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
    }

    public void rotateCollection(double position, double timeoutS) {

        while (((LinearOpMode) theOpMode).opModeIsActive() && (runtime.seconds() < timeoutS)) {
            rotatorServo.setPosition(position);
        }
    }

    public void rotateCollectionStart(double position, double timeoutS) {
        rotatorServo.setPosition(position);
    }

    public boolean rotateCollectionCheck(double position, double timeoutS) {

        if (((LinearOpMode) theOpMode).opModeIsActive() && (runtime.seconds() < timeoutS)) {
            rotatorServo.setPosition(position);
            return true;

        }
        else {
            rotateCollectionEnd();
            return false;
        }

    }
    public void rotateCollectionEnd() {
    }

    public void collectionTeleop() {

    }

    public void setCollectionPosition() {
        if (theOpMode.gamepad2.left_bumper) {
            rotatorServo.setPosition(.4);
        } else if (theOpMode.gamepad2.right_bumper) {
            rotatorServo.setPosition(0.6);
        }
        if (theOpMode.gamepad2.dpad_left) {
            rotServo.setPosition(.35);
        } else if (theOpMode.gamepad2.dpad_right) {
            rotServo.setPosition(0.05);
        }
        if (theOpMode.gamepad2.dpad_up) {
            imAboutToDie.setPosition(0);
        }
        else if (theOpMode.gamepad2.dpad_down) {
            imAboutToDie.setPosition(.9);
        }
    }
    public void moveClaw(double clawPos, double timeoutS) {
        while (((LinearOpMode) theOpMode).opModeIsActive() && (runtime.seconds() < timeoutS)) {
            imAboutToDie.setPosition(clawPos);
        }

    }
    public void rotateArm(double armPos, double timeoutS) {
        while (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS) {
            rotatorServo.setPosition(armPos);
        }
    }
    public void rotateArmStart(double armPos, double timeoutS) {
        rotatorServo.setPosition(armPos);
    }
    public boolean rotateArmCheck(double armPos, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS) {
            rotatorServo.setPosition(armPos);
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
}
