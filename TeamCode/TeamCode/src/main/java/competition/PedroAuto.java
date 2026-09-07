package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooting {

    private final DcMotorEx shooterMotor;
    private final DcMotorEx riserMotor;

    private static final double MAX_RPM = 6000.0;
    private static final double NORMAL_RPM = MAX_RPM * 0.50;
    private static final double BOOST_RPM = MAX_RPM;

    private static final double RAMP_RATE = 12000.0;

    private double targetRPM = NORMAL_RPM;
    private double currentTargetRPM = NORMAL_RPM;

    public Shooting(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(
                DcMotorEx.class,
                RobotMap.SHOOTER_MOTOR
        );

        riserMotor = hardwareMap.get(
                DcMotorEx.class,
                RobotMap.RISER_MOTOR
        );

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        riserMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        shooterMotor.setVelocity(rpmToTicksPerSecond(NORMAL_RPM));
        riserMotor.setPower(0);
    }

    public void update() {
        double targetTicks = rpmToTicksPerSecond(targetRPM);
        double currentTicks = rpmToTicksPerSecond(currentTargetRPM);

        double difference = targetTicks - currentTicks;

        if (Math.abs(difference) <= RAMP_RATE) {
            currentTargetRPM = targetRPM;
        } else if (difference > 0) {
            currentTargetRPM += ticksPerSecondToRPM(RAMP_RATE);
        } else {
            currentTargetRPM -= ticksPerSecondToRPM(RAMP_RATE);
        }

        currentTargetRPM = Math.max(
                0,
                Math.min(currentTargetRPM, MAX_RPM)
        );

        shooterMotor.setVelocity(
                rpmToTicksPerSecond(currentTargetRPM)
        );
    }

    public void shoot() {
        targetRPM = NORMAL_RPM;
    }

    public void boost() {
        targetRPM = BOOST_RPM;
    }

    public void reverse(double power) {
        power = Math.max(0.0, Math.min(power, 1.0));

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setPower(-power);
    }

    public void riser(double power) {
        power = Math.max(-1.0, Math.min(power, 1.0));
        riserMotor.setPower(power);
    }

    public boolean isReady() {
        double actualRPM = getRPM();

        return Math.abs(actualRPM - currentTargetRPM) <= 150;
    }

    public double getRPM() {
        double ticksPerSecond = shooterMotor.getVelocity();

        return ticksPerSecondToRPM(ticksPerSecond);
    }

    public double getTargetRPM() {
        return currentTargetRPM;
    }

    public double getMaxRPM() {
        return MAX_RPM;
    }

    private double rpmToTicksPerSecond(double rpm) {
        double ticksPerRev =
                shooterMotor.getMotorType().getTicksPerRev();

        return (rpm * ticksPerRev) / 60.0;
    }

    private double ticksPerSecondToRPM(double ticksPerSecond) {
        double ticksPerRev =
                shooterMotor.getMotorType().getTicksPerRev();

        return (ticksPerSecond * 60.0) / ticksPerRev;
    }

    public void stop() {
        targetRPM = 0;
        currentTargetRPM = 0;

        shooterMotor.setVelocity(0);
        riserMotor.setPower(0);
    }
}