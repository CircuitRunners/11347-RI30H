package util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PIDController {
    private final double kP;
    private final double kD;
    private double previousError = 0;

    public PIDController(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
    }

    public double calculate(double target, double current, double dt) {
        double error = AngleUnit.normalizeRadians(target - current);
        if (dt <= 0) dt = 0.001;
        double derivative = (error - previousError) / dt;
        previousError = error;
        return (error * kP) + (derivative * kD);
    }

    public void reset() {
        previousError = 0;
    }
}