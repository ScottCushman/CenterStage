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
        imAboutToDie = hardwareMap.get(Servo.class, "armServo");
        imGoingToDie = hardwareMap.get(Servo.class, "arm");
        hangMotor = hardwareMap.get(DcMotor.class, "hang");

        rotatorServo.setDirection(Servo.Direction.FORWARD);
        imGoingToDie.setDirection(Servo.Direction.REVERSE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imGoingToDie1 = imGoingToDie2;
        imAboutToDie1 = imAboutToDie2;
     //   colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
    }

    public void collectionTeleop() {

    }

    public void setCollectionPosition() {
        if (theOpMode.gamepad1.left_bumper || theOpMode.gamepad2.right_bumper) {
            rotatorServo.setPosition(.5);
        } else if (theOpMode.gamepad1.right_bumper || theOpMode.gamepad2.left_bumper) {
            rotatorServo.setPosition(0.8);
        }

        if (theOpMode.gamepad2.dpad_left) {
            rotServo.setPosition(0);
        } else if (theOpMode.gamepad2.dpad_right) {
            rotServo.setPosition(0.2);
        }
        if (theOpMode.gamepad2.dpad_up) {
            imAboutToDie.setPosition(.8);
            imGoingToDie.setPosition(.8);
        }
        else if (theOpMode.gamepad2.dpad_down) {
            imAboutToDie.setPosition(.18);
            imGoingToDie.setPosition(.18);
        }
        if (theOpMode.gamepad2.y) {
            imAboutToDie.setPosition(.5);
            imGoingToDie.setPosition(.5);

        }
        if (theOpMode.gamepad2.x) {
            imAboutToDie.setPosition(.25);
            imGoingToDie.setPosition(.25);

        }
    }
    public void moveClaw(double clawPos, double timeoutS) {
        while (((LinearOpMode) theOpMode).opModeIsActive() && (runtime.seconds() < timeoutS)) {
            rotatorServo.setPosition(clawPos);
        }

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
}
