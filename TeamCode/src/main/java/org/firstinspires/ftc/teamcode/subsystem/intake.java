package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class intake {
    public DcMotor intakeMotor;
    public CRServo intakeServo;
    public boolean toggle = false;
    public boolean intakeServoToggle = false;

    public void init(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeServo = hardwareMap.get(CRServo.class, "transfer");
    }

    public void setPower(double p){
        intakeMotor.setPower(p);
    }
    public void servoSetState(boolean s){intakeServoToggle = s;}

    public void tick(){
            if(toggle){
                intakeMotor.setPower(0.9);
            }else{
                intakeMotor.setPower(-0.3);
            }

            if(intakeServoToggle){
                intakeServo.setPower(1.0);
            }else{
                intakeServo.setPower(-1.0);
            }
    }

    public Command toggle(){
        if(toggle){
            toggle = false;
            servoSetState(false);
        }else{
            toggle = true;
            servoSetState(true);
        }
        return null;
    }
}
