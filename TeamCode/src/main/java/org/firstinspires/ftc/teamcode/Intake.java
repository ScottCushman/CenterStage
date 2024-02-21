package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



public class Intake {
    private DcMotor intakeMotor;

    public Intake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class, "hang");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





    }
    public void spin(long seconds, boolean in) throws InterruptedException {
        if (in) {
            intakeMotor.setPower(-1);
            sleep(seconds);

        } else {
            intakeMotor.setPower(1);
            sleep(seconds);

        }

    }
    public void stop(){
        intakeMotor.setPower(0);
    }
}
