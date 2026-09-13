package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {
    public DcMotor intakeMotor;
    public boolean toggle = false;

    public void init(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
    }

    public void setPower(double p){
        intakeMotor.setPower(p);
    }

    public void tick(){
            if(toggle){
                intakeMotor.setPower(-0.8);
            }else{
                intakeMotor.setPower(0.3);
            }
    }

    public Command toggle(){
        if(toggle){
            toggle = false;
        }else{
            toggle = true;
        }
        return null;
    }
}
