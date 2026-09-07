package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooting {

    private final DcMotorEx shooterMotor;
    private final DcMotorEx riserMotor;

    private static final double NORMAL_POWER = 0.00;
    private static final double BOOST_POWER = 1.00;

    public Shooting(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, RobotMap.SHOOTER_MOTOR);
        riserMotor = hardwareMap.get(DcMotorEx.class, RobotMap.RISER_MOTOR);

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        riserMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter ALWAYS starts at 50%
        shooterMotor.setPower(NORMAL_POWER);

        // Riser starts off
        riserMotor.setPower(0);
    }

    // Normal shooter power: 50%
    public void shoot() {
        shooterMotor.setPower(NORMAL_POWER);
    }

    // Boost shooter to 100%
    public void boost() {
        shooterMotor.setPower(BOOST_POWER);
    }

    // Riser control
    public void riser(double power) {
        power = Math.max(-1.0, Math.min(power, 1.0));
        riserMotor.setPower(power);
    }

    // Reverse shooter
    public void reverse(double power) {
        power = Math.max(0.0, Math.min(power, 1.0));
        shooterMotor.setPower(-power);
    }

    // Stop both motors
    public void stop() {
        shooterMotor.setPower(0);
        riserMotor.setPower(0);
    }
}