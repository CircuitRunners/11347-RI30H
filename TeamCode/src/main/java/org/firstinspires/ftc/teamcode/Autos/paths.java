package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Config.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.intake;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

@Autonomous(name = "Pollen Nectar AprilTag Auto")
public class paths extends OpMode {

    private final MecanumDrive drive = new MecanumDrive();
    private final intake intake = new intake();

    private Limelight3A limelight;

    private DcMotorEx shooterMotor;
    private DcMotorEx riserMotor;

    private final Set<Integer> validTags = new HashSet<>();

    private boolean intakeRunning = false;

    private int state = 0;
    private long stateStart = 0;

    private static final int FIND_POLLEN = 0;
    private static final int DRIVE_TO_POLLEN = 1;
    private static final int INTAKE_POLLEN = 2;

    private static final int FIND_NECTAR = 3;
    private static final int DRIVE_TO_NECTAR = 4;
    private static final int INTAKE_NECTAR = 5;

    private static final int FIND_APRILTAG = 6;
    private static final int AIM_AT_APRILTAG = 7;
    private static final int SHOOT = 8;
    private static final int FINISHED = 9;

    private static final double TX_DEADBAND = 2.0;
    private static final double ROTATE_GAIN = 0.025;
    private static final double MAX_ROTATE = 0.45;

    private static final double TARGET_TY = 8.0;

    private static final double INTAKE_SPEED = 0.40;
    private static final double SHOOTER_POWER = 0.75;

    private static final long INTAKE_TIME = 1200;
    private static final long SHOOT_TIME = 1800;

    @Override
    public void init() {

        drive.init(hardwareMap);
        intake.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        riserMotor = hardwareMap.get(DcMotorEx.class, "riserMotor");

        shooterMotor.setPower(0);
        riserMotor.setPower(0);

        validTags.add(30);
        validTags.add(31);
        validTags.add(32);
        validTags.add(33);
        validTags.add(34);
        validTags.add(35);
        validTags.add(36);
        validTags.add(37);
        validTags.add(38);
        validTags.add(39);
        validTags.add(40);
        validTags.add(41);
        validTags.add(42);
        validTags.add(43);
        validTags.add(44);
        validTags.add(45);

        limelight.pipelineSwitch(0);
        limelight.start();

        state = FIND_POLLEN;
        stateStart = System.currentTimeMillis();
    }

    @Override
    public void start() {

        limelight.pipelineSwitch(0);

        state = FIND_POLLEN;
        stateStart = System.currentTimeMillis();
    }

    @Override
    public void loop() {

        intake.tick();

        switch (state) {

            case FIND_POLLEN:

                stopDrive();

                if (detectPollen()) {
                    state = DRIVE_TO_POLLEN;
                }

                break;

            case DRIVE_TO_POLLEN:

                if (reachedGameObject()) {

                    stopDrive();
                    intakeOn();

                    state = INTAKE_POLLEN;
                    stateStart = System.currentTimeMillis();
                }

                break;

            case INTAKE_POLLEN:

                stopDrive();

                if (System.currentTimeMillis() - stateStart >= INTAKE_TIME) {

                    intakeOff();

                    limelight.pipelineSwitch(1);

                    state = FIND_NECTAR;
                    stateStart = System.currentTimeMillis();
                }

                break;

            case FIND_NECTAR:

                stopDrive();

                if (detectNectar() != null) {
                    state = DRIVE_TO_NECTAR;
                }

                break;

            case DRIVE_TO_NECTAR:

                if (reachedGameObject()) {

                    stopDrive();
                    intakeOn();

                    state = INTAKE_NECTAR;
                    stateStart = System.currentTimeMillis();
                }

                break;

            case INTAKE_NECTAR:

                stopDrive();

                if (System.currentTimeMillis() - stateStart >= INTAKE_TIME) {

                    intakeOff();

                    limelight.pipelineSwitch(2);

                    state = FIND_APRILTAG;
                    stateStart = System.currentTimeMillis();
                }

                break;

            case FIND_APRILTAG:

                stopDrive();

                if (findValidAprilTag() != null) {
                    state = AIM_AT_APRILTAG;
                }

                break;

            case AIM_AT_APRILTAG:

                LLResultTypes.FiducialResult tag = findValidAprilTag();

                if (tag == null) {

                    stopDrive();
                    state = FIND_APRILTAG;

                    break;
                }

                double tx = tag.getTargetXDegrees();

                if (Math.abs(tx) <= TX_DEADBAND) {

                    stopDrive();

                    shooterMotor.setPower(SHOOTER_POWER);
                    riserMotor.setPower(SHOOTER_POWER);

                    state = SHOOT;
                    stateStart = System.currentTimeMillis();

                } else {

                    double rotate = tx * ROTATE_GAIN;

                    if (rotate > MAX_ROTATE) {
                        rotate = MAX_ROTATE;
                    }

                    if (rotate < -MAX_ROTATE) {
                        rotate = -MAX_ROTATE;
                    }

                    drive.drive(0, 0, rotate);
                }

                break;

            case SHOOT:

                stopDrive();

                if (System.currentTimeMillis() - stateStart >= SHOOT_TIME) {

                    shooterMotor.setPower(0);
                    riserMotor.setPower(0);

                    state = FINISHED;
                }

                break;

            case FINISHED:

                stopDrive();
                intakeOff();

                shooterMotor.setPower(0);
                riserMotor.setPower(0);

                break;
        }

        sendTelemetry();
    }

    private boolean detectPollen() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return false;
        }

        List<LLResultTypes.DetectorResult> detectors =
                result.getDetectorResults();

        for (LLResultTypes.DetectorResult detector : detectors) {

            String name = detector.getClassName();

            if (name != null &&
                    name.equalsIgnoreCase("Pollen")) {

                return true;
            }
        }

        return false;
    }

    private String detectNectar() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.DetectorResult> detectors =
                result.getDetectorResults();

        for (LLResultTypes.DetectorResult detector : detectors) {

            String name = detector.getClassName();

            if (name == null) {
                continue;
            }

            if (name.equalsIgnoreCase("Red") ||
                    name.equalsIgnoreCase("Red Nectar")) {

                return "RED";
            }

            if (name.equalsIgnoreCase("Blue") ||
                    name.equalsIgnoreCase("Blue Nectar")) {

                return "BLUE";
            }
        }

        return null;
    }

    private boolean reachedGameObject() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {

            stopDrive();
            return false;
        }

        double tx = result.getTx();
        double ty = result.getTy();

        if (Math.abs(tx) > TX_DEADBAND) {

            double rotate = tx * ROTATE_GAIN;

            if (rotate > MAX_ROTATE) {
                rotate = MAX_ROTATE;
            }

            if (rotate < -MAX_ROTATE) {
                rotate = -MAX_ROTATE;
            }

            drive.drive(0, 0, rotate);

            return false;
        }

        if (ty < TARGET_TY) {

            drive.drive(INTAKE_SPEED, 0, 0);

            return false;
        }

        stopDrive();

        return true;
    }

    private LLResultTypes.FiducialResult findValidAprilTag() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        LLResultTypes.FiducialResult bestTag = null;

        for (LLResultTypes.FiducialResult tag : tags) {

            int id = tag.getFiducialId();

            if (!validTags.contains(id)) {
                continue;
            }

            if (bestTag == null ||
                    tag.getTargetArea() > bestTag.getTargetArea()) {

                bestTag = tag;
            }
        }

        return bestTag;
    }

    private void intakeOn() {

        if (!intakeRunning) {

            intake.toggle();
            intakeRunning = true;
        }
    }

    private void intakeOff() {

        if (intakeRunning) {

            intake.toggle();
            intakeRunning = false;
        }
    }

    private void stopDrive() {

        drive.drive(0, 0, 0);
    }

    private void sendTelemetry() {

        telemetry.addData("STATE", state);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            telemetry.addData("TX", "%.2f", result.getTx());
            telemetry.addData("TY", "%.2f", result.getTy());
            telemetry.addData("TA", "%.2f", result.getTa());

            LLResultTypes.FiducialResult tag = findValidAprilTag();

            if (tag != null) {

                telemetry.addData(
                        "APRILTAG",
                        tag.getFiducialId()
                );

                telemetry.addData(
                        "TAG TX",
                        "%.2f",
                        tag.getTargetXDegrees()
                );
            }
        }

        telemetry.update();
    }

}
