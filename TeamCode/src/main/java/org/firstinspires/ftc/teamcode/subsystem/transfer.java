package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class transfer {
    CRServo transferServo;
    private boolean transferServoToggle = false;

    public void init(HardwareMap hardwareMap){
        transferServo = hardwareMap.get(CRServo.class, "transfer");
    }

    public void setTransferServoToggle(boolean b){transferServoToggle = b;}

    public void tick(){
        if(transferServoToggle){
            transferServo.setPower(1.0);
        }else{
            transferServo.setPower(0.0);
        }
    }
}
