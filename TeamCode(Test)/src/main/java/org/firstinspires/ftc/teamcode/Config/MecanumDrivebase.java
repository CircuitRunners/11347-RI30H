package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import util.PIDController;

public class MecanumDrivebase {

    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;

    private final IMU imu;

    private double maxPower = 1.00;
    private static final double MIN_POWER = 0.01;
    private static final double MAX_POWER = 1.00;

    private boolean threeYearOldMode = false;
    private static final double THREE_YEAR_OLD_MAX_POWER = 0.30;
    private static final double THREE_YEAR_OLD_ROTATION = 0.30;

    private static final double PRECISION_MULTIPLIER = 0.25;
    private static final double STICK_DEADZONE = 0.08;
    private static final double ROTATION_MULTIPLIER = 0.65;

    private static final double SMOOTHING_RATE = 10.0;
    private double smoothForward = 0;
    private double smoothRight = 0;
    private double smoothRotation = 0;
    private final ElapsedTime timer = new ElapsedTime();

    private boolean headingHold = false;
    private double targetHeading = 0;
    private final PIDController headingController = new PIDController(1.8, 0.05);

    public MecanumDrivebase(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, RobotMap.FL_MOTOR);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, RobotMap.FR_MOTOR);
        backLeftMotor = hardwareMap.get(DcMotorEx.class, RobotMap.BL_MOTOR);
        backRightMotor = hardwareMap.get(DcMotorEx.class, RobotMap.BR_MOTOR);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx[] motors = {
                frontLeftMotor, frontRightMotor,
                backLeftMotor, backRightMotor
        };

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }

        imu = hardwareMap.get(IMU.class, RobotMap.IMU);
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        timer.reset();
    }

    public void drive(
            double forward,
            double right,
            double rotate,
            boolean precisionMode
    ) {
        forward = applyDeadzone(forward);
        right = applyDeadzone(right);
        rotate = applyDeadzone(rotate);

        if (threeYearOldMode) {
            rotate *= THREE_YEAR_OLD_ROTATION;
        } else {
            rotate *= ROTATION_MULTIPLIER;
        }

        double dt = timer.seconds();
        timer.reset();

        smoothForward = smooth(smoothForward, forward, dt);
        smoothRight = smooth(smoothRight, right, dt);
        smoothRotation = smooth(smoothRotation, rotate, dt);

        forward = smoothForward;
        right = smoothRight;
        rotate = smoothRotation;

        if (headingHold) {
            double currentHeading = getHeading();

            if (Math.abs(rotate) > 0.0) {
                targetHeading = currentHeading;
                headingController.reset();
            } else {
                rotate = headingController.calculate(targetHeading, currentHeading, dt);
            }
        }

        double frontLeft = forward + right + rotate;
        double frontRight = forward - right - rotate;
        double backLeft = forward - right + rotate;
        double backRight = forward + right - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeft), Math.max(Math.abs(frontRight), Math.max(Math.abs(backLeft), Math.abs(backRight)))));

        frontLeft /= max;
        frontRight /= max;
        backLeft /= max;
        backRight /= max;

        double power = maxPower;

        if (threeYearOldMode) {
            power = Math.min(power, THREE_YEAR_OLD_MAX_POWER);
        }

        if (precisionMode) {
            power *= PRECISION_MULTIPLIER;
        }

        frontLeftMotor.setPower(frontLeft * power);
        frontRightMotor.setPower(frontRight * power);
        backLeftMotor.setPower(backLeft * power);
        backRightMotor.setPower(backRight * power);
    }

    private double applyDeadzone(double value) {
        if (Math.abs(value) < STICK_DEADZONE) {
            return 0;
        }
        double sign = Math.signum(value);
        double magnitude = (Math.abs(value) - STICK_DEADZONE) / (1.0 - STICK_DEADZONE);
        return sign * magnitude;
    }

    private double smooth(double current, double target, double dt) {
        double smoothingFactor = Math.min(1.0, dt * SMOOTHING_RATE);
        return current + ((target - current) * smoothingFactor);
    }

    public void toggleHeadingHold() {
        headingHold = !headingHold;
        if (headingHold) {
            targetHeading = getHeading();
            headingController.reset();
        }
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void toggleThreeYearOldMode() {
        threeYearOldMode = !threeYearOldMode;
    }

    public boolean isThreeYearOldMode() {
        return threeYearOldMode;
    }

    public void increasePower() {
        maxPower += 0.10;
        if (maxPower > MAX_POWER) maxPower = MAX_POWER;
    }

    public void decreasePower() {
        maxPower -= 0.10;
        if (maxPower < MIN_POWER) maxPower = MIN_POWER;
    }

    public double getMaxPower() {
        return maxPower;
    }
}