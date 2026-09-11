

package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config.MecanumDrive;


//@Disabled
@TeleOp(name = "Biobuzz TeleOp")
public class MainTeleOp extends OpMode {
    private MecanumDrive drive;

    @Override
    public void init(){
        drive = new MecanumDrive();
        drive.init(hardwareMap);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {



        double forward = -1*gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double rotate  =  gamepad1.right_stick_x;
        drive.drive(forward, strafe, rotate);




        telemetry.update();
    }
}