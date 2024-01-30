package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    private DcMotor intakeMotor;
    public Intake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class, "hang");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
    public void spin(double seconds, boolean in){
        if (in) {
            intakeMotor.setPower(-1);




        } else {
            intakeMotor.setPower(1);
        }

    }
    public void stop(){
        intakeMotor.setPower(0);
    }
}
