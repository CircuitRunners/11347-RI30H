package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class outtake{
    // --- Hardware ---
    private DcMotorEx shooter;


    // --- Shooter Constants ---
    private static double TARGET_RPM = 0;         // desired shooter RPM
    private static double MOTOR_RPM = 6000;          // motor RPM (based on max motor rpm)
    private static double GEAR_RATIO = 1;            // gear ratio from motor to shooter
    private static double TICKS_PER_REV = 28;       // motor encoder ticks per revolution
    private boolean active;

    //--- hood caculator pid values ---
    private static final double GRAVITY_IN_PER_S2 = 386.09; // inches per second squared
    private static final double VELOCITY_CONSTANT = 0.025;

    // --- PIDF Coefficients ---
    //working value 35 on October 9th, 2025
    /*    public double kP = 35.0;
    public double kI = 0.0;
    public double kD = 10.0;
    public double kF = 13.0;*/
    public static double kP = 35.0;
    public static double kI = 0.0;
    public static double kD = 10.0;
    public static double kF = 13.0;

    public outtake(HardwareMap hardwareMap, Telemetry telemetry, double defaultTargetRPM,
                   double defaultMotorRPM, double defaultGearRatio, double defaultTicks, String motorName) {
        // Configs shooter
        shooter = hardwareMap.get(DcMotorEx.class, motorName);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Configs defaults
        setTargetRPM(defaultTargetRPM);
        setMotorRPM(defaultMotorRPM);
        setGearRatio(defaultGearRatio);
        setTicksPerRev(defaultTicks);
        active = Math.abs(getTargetRPM()) > 0;

        // Apply initial PIDF coefficients
        applyPIDF();

        telemetry.addLine("shooter Init Done");
    }

    public void setShooterPIDF(double kf, double kp, double kd, double ki) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        applyPIDF();
    }

    /** Applies current shooter velocity PIDF coefficients */
    public void applyPIDF() {
        shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    // --- Constants Control ---
    /**
     * Changes the target RPM of the shooter
     * @param targetRPM Set to the target RPM of the shooter
     */
    public void setTargetRPM(double targetRPM) {
        TARGET_RPM = targetRPM;
    }

    /**
     * Returns the target RPM of the shooter, used to check if velo
     * is within tolerance
     * @return returns the target RPM of the shooter
     */
    public double getTargetRPM() {
        return TARGET_RPM;
    }

    /**
     * Changes the RPM of the motor
     * @param motorRPM Set to the RPM of the motor
     *
     */
    public void setMotorRPM(double motorRPM) {
        MOTOR_RPM = motorRPM;
    }

    /**
     * Changes the gear ratio between the motor and the shooter
     * @param gearRatio Set to the gear ratio used between the
     *                  motor and shooter
     *      1.0 is a 1:1 gear ratio
     *      2.5 is a 2.5:1 gear increase
     *      0.5 is a 0.5:1 gear reduction
     */
    public void setGearRatio(double gearRatio) {
        GEAR_RATIO = gearRatio;
    }

    /**
     * Returns the current gear ratio of the shooter
     * @return returns the current GEAR_RATIO of the shooter system
     */
    public double getGearRatio() {
        return GEAR_RATIO;
    }

    /**
     * Changes the Ticks Per Revolution of the motor
     * Called Encoder Resolution on gobilda website
     * @param TicksPerRev Set to the Ticks per rev of the motor
     *                    being used
     */
    public void setTicksPerRev(double TicksPerRev) {
        TICKS_PER_REV = TicksPerRev;
    }

    /**
     * Returns the current Ticks Per Rev of the shooter
     * @return returns the TICKS_PER_REV of the shooter flywheel
     */
    public double getTicksPerRev() {
        return TICKS_PER_REV;
    }

    /**
     * Calculates ticks per second based on target RPM
     * Sets the target velocity
     * */
    public void update() {
        double targetTicksPerSec = ((TARGET_RPM / GEAR_RATIO) * TICKS_PER_REV) / 60;
        shooter.setVelocity(targetTicksPerSec);

        active = Math.abs(getTargetRPM()) > 0;
    }

    /** Stops all shooter motion immediately. */
    public void eStop() {
        shooter.setPower(0);
        shooter.setVelocity(0);
    }

    /**
     * Gets shooter current velocity
     * @return Returns current shooter RPM based on the
     *         motor rpm, ticks per rev, and gear ratio
     */
    public double getShooterVelocity() {
        double currTicksPerSec = shooter.getVelocity(); // ticks/s of motor
        double currMotorRPM = (currTicksPerSec * 60.0) / TICKS_PER_REV;
        double currShooterRPM = currMotorRPM * GEAR_RATIO;

        return currShooterRPM;  
    }

    public double getMotorVoltage() {
        return shooter.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isActive() {
        return active;
    }

    public boolean isAtTargetThreshold() {
        return ((getShooterVelocity() > (getTargetRPM() - 200)) && (getShooterVelocity() < (getTargetRPM() + 100)) && getShooterVelocity() != 0);
    }
}

