package org.firstinspires.ftc.teamcode.TeleOps.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Start with kF auto, then raise/lower kF until the motor reaches near target at steady state (low error).
 * Add small kP to tighten response (too high = oscillation).
 * Only add kI if you see steady-state error that kF+kP can’t fix, prob won't be needed
 * Add kD to damp oscillations during spin-up, might not be needed
 */

@TeleOp(name="ShooterPIDTest - POLLEN")
public class ShooterPIDTestPollen extends OpMode {
    private DcMotorEx shooter;

    // Dashboard-tunable constants
    public static double TARGET_RPM = 0.0; // 4000
    public static double MOTOR_RPM = 6000.0; // 1410
    public static double GEAR_RATIO = 1.0;
    public static double TICKS_PER_REV = 28.0;


    // PIDF (velocity)
    public static double kP = 35.0; //15.0
    public static double kI = 0.0;
    public static double kD = 10.0;
    public static double kF = 13.0; //13.7

    public double kfValue;

    // Toggles shooter on and off in dashboard
    public static boolean runShooter = false;


    @Override
    public void init() {

        shooter = hardwareMap.get(DcMotorEx.class, "pollenShooter");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        applyPIDF();

        telemetry.addLine("init done");
        telemetry.update();
    }

    //TODO: test if this works, if not remove
    private void applyPIDF() {
        double kFLocal = kF;
        if (kFLocal == 0.0) {
            // Based on achievable max ticks/s reported by the SDK
            // Good starting point for later tuning in Dashboard.
            double maxTps = shooter.getMotorType().getAchieveableMaxTicksPerSecond();
            if (maxTps <= 0) maxTps = (1620.0 * TICKS_PER_REV) / 60.0;
            // REV internal scaling expects kF around 32767/maxVelocity as a reasonable baseline
            kFLocal = 32767.0 / maxTps;
        }
        kfValue = kFLocal;
        shooter.setVelocityPIDFCoefficients(kP, kI, kD, kFLocal);
    }

    @Override
    public void loop() {
        /** Live PIDF updates via dashboard */
        applyPIDF();

        double targetMotorRPM = TARGET_RPM / GEAR_RATIO;
        double targetTicksPerSec = (targetMotorRPM * TICKS_PER_REV) / 60.0;

        if (runShooter) {
            shooter.setVelocity(targetTicksPerSec); // ticks/s
        } else {
            shooter.setVelocity(0);
        }

        // Gets current velo of motor
        double currTicksPerSec = shooter.getVelocity(); // ticks/s of motor
        double currMotorRPM = (currTicksPerSec * 60.0) / TICKS_PER_REV;
        double currShooterRPM = currMotorRPM * GEAR_RATIO;

        // Tuning Stuff
        double errorMotorRPM = targetMotorRPM - currMotorRPM;
        double errorShooterRPM = TARGET_RPM - currShooterRPM;

        // ===== Graphs on FTC Dashboard =====
        telemetry.addData("target_shooter_rpm", TARGET_RPM);
        telemetry.addData("current_shooter_rpm", currShooterRPM);
        telemetry.addData("target_motor_rpm", targetMotorRPM);
        telemetry.addData("current_motor_rpm", currMotorRPM);
        telemetry.addData("error_shooter_rpm", errorShooterRPM);
        telemetry.addData("error_motor_rpm", errorMotorRPM);
        telemetry.addData("motor_ticks_per_sec", currTicksPerSec);
        telemetry.addData("kF Local Value: ", kfValue);

    }
}
