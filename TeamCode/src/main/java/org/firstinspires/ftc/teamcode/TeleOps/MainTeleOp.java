

package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.intake;
import org.firstinspires.ftc.teamcode.subsystem.outtake;


//@Disabled
@TeleOp(name = "Biobuzz TeleOp - RBI30H")
public class MainTeleOp extends OpMode {
    private MecanumDrive drive;
    private intake intake;
    private outtake outtakePollen;

    private outtake outtakeNectar;
    double GamepadLeftY;
    double GamepadLeftX;
    double r;
    double scale;
    double exponent = 3;

    @Override
    public void init(){
        drive = new MecanumDrive();
        drive.init(hardwareMap);

        outtakePollen = new outtake(hardwareMap, telemetry, 500, 500, 1, 28, "pollenShooter");
        outtakeNectar = new outtake(hardwareMap, telemetry, 500, 500, 1, 28, "nectarShooter");

        intake = new intake();
        intake.init(hardwareMap);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        GamepadLeftY = gamepad1.left_stick_y;
        GamepadLeftX = gamepad1.left_stick_x;
        r = Math.sqrt(Math.pow(GamepadLeftX, 2) + Math.pow(GamepadLeftY, 2));
        scale = Math.max(r, 0.01) / (Math.max(Math.max(Math.abs(GamepadLeftX), Math.abs(GamepadLeftY)), 0.01));
        GamepadLeftY = GamepadLeftY * scale;
        GamepadLeftX = GamepadLeftX * scale;
        double forward = (Math.abs(Math.pow(GamepadLeftY, exponent))) * Math.signum(GamepadLeftY);
        double strafe = (Math.abs(Math.pow(GamepadLeftX, exponent))) * Math.signum(GamepadLeftX);
        double rotate  =  gamepad1.right_stick_x;
        drive.drive(forward, strafe, rotate);

        if(gamepad1.left_trigger > 0.5){
            intake.setPower(0.9);
        }else if(gamepad1.right_trigger > 0.5){
            intake.setPower(-0.7);
        }else{
            intake.setPower(0.0);
        }


        if(gamepad1.right_bumper){
            outtakePollen.setTargetRPM(4000.0);
        }else{
            outtakePollen.setTargetRPM(500.0);
        }

        if(gamepad1.left_bumper){
            outtakeNectar.setTargetRPM(4000.0);
        }else{
            outtakeNectar.setTargetRPM(500.0);
        }

        telemetry.update();
    }
}