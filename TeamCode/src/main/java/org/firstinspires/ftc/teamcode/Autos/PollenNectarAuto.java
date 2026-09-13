package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.intake;

import java.util.List;

@Autonomous(name = "Pollen Nectar AUTO")
public class paths extends OpMode {


    private Follower follower;
    private final intake intake = new intake();

    private Limelight3A limelight;

    private DcMotor shooterMotor;
    private DcMotor riserMotor;

    private final Pose start = new Pose(56, 8, Math.toRadians(90));

    private final Pose path1End = new Pose(55.9936, 19.2415, Math.toRadians(270));
    private final Pose path1Control = new Pose(55.9936, 19.8479, 0);

    private final Pose point2 = new Pose(8.4079, 23.5486, Math.toRadians(270));
    private final Pose point2Control1 = new Pose(28.5686, 21.5407, 0);
    private final Pose point2Control2 = new Pose(5.7521, 42.4993, 0);

    private final Pose point3 = new Pose(8.7907, 7.6743, Math.toRadians(270));

    private final Pose point4 = new Pose(57.8129, 116.4722, Math.toRadians(90));
    private final Pose point4Control1 = new Pose(56.6, 46.1265, 0);
    private final Pose point4Control2 = new Pose(5.5364, 112.525, 0);

    private PathChain path1;
    private PathChain path2;
    private PathChain path3;
    private PathChain path4;

    private int state = 0;
    private long stateStart = 0;
    private boolean intakeRunning = false;

    private static final int DRIVE_TO_POLLEN = 0;
    private static final int FIND_POLLEN = 1;
    private static final int INTAKE_POLLEN = 2;
    private static final int SHOOT_POLLEN = 3;
    private static final int DRIVE_TO_NECTAR = 4;
    private static final int FIND_NECTAR = 5;
    private static final int INTAKE_NECTAR = 6;
    private static final int SHOOT_NECTAR = 7;
    private static final int FINISHED = 8;

    @Override
    public void init() {

        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        intake.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        riserMotor = hardwareMap.get(DcMotor.class, "riserMotor");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        riserMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setPower(0);
        riserMotor.setPower(0);

        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(start, path1Control, path1End))
                .setLinearHeadingInterpolation(
                        start.getHeading(),
                        path1End.getHeading()
                )
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        path1End,
                        point2Control1,
                        point2Control2,
                        point2
                ))
                .setConstantHeadingInterpolation(point2.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(point2, point3))
                .setConstantHeadingInterpolation(point3.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        point3,
                        point4Control1,
                        point4Control2,
                        point4
                ))
                .setLinearHeadingInterpolation(
                        point3.getHeading(),
                        point4.getHeading()
                )
                .build();

        follower.setPose(start);

        limelight.pipelineSwitch(0);
        limelight.start();

        state = DRIVE_TO_POLLEN;
    }

    @Override
    public void start() {

        follower.followPath(path1);

        stateStart = System.currentTimeMillis();
    }

    @Override
    public void loop() {

        follower.update();
        intake.tick();
        Scheduler.execute();

        long elapsed = System.currentTimeMillis() - stateStart;

        switch (state) {

            case DRIVE_TO_POLLEN:

                if (!follower.isBusy()) {

                    limelight.pipelineSwitch(0);

                    state = FIND_POLLEN;
                    stateStart = System.currentTimeMillis();
                }

                break;

            case FIND_POLLEN:

                if (detectPollen()) {

                    intakeOn();

                    state = INTAKE_POLLEN;
                    stateStart = System.currentTimeMillis();
                }

                break;

            case INTAKE_POLLEN:

                if (elapsed >= 1200) {

                    intakeOff();
                    shooterOn();

                    state = SHOOT_POLLEN;
                    stateStart = System.currentTimeMillis();
                }

                break;

            case SHOOT_POLLEN:

                if (elapsed >= 1800) {

                    shooterOff();

                    follower.followPath(path2);

                    state = DRIVE_TO_NECTAR;
                    stateStart = System.currentTimeMillis();
                }

                break;

            case DRIVE_TO_NECTAR:

                if (!follower.isBusy()) {

                    limelight.pipelineSwitch(1);

                    state = FIND_NECTAR;
                    stateStart = System.currentTimeMillis();
                }

                break;

            case FIND_NECTAR:

                String nectar = detectNectar();

                if (nectar != null) {

                    telemetry.addData("NECTAR", nectar);

                    intakeOn();

                    state = INTAKE_NECTAR;
                    stateStart = System.currentTimeMillis();
                }

                break;

            case INTAKE_NECTAR:

                if (elapsed >= 1200) {

                    intakeOff();
                    shooterOn();

                    state = SHOOT_NECTAR;
                    stateStart = System.currentTimeMillis();
                }

                break;

            case SHOOT_NECTAR:

                if (elapsed >= 1800) {

                    shooterOff();

                    follower.followPath(path3);

                    state = FINISHED;
                    stateStart = System.currentTimeMillis();
                }

                break;

            case FINISHED:

                intakeOff();
                shooterOff();

                break;
        }

        telemetry.addData("STATE", state);
        telemetry.addData("POLLEN", detectPollen());
        telemetry.addData("NECTAR", detectNectar());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData(
                "HEADING",
                Math.toDegrees(follower.getPose().getHeading())
        );

        telemetry.update();
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

            if (name.equalsIgnoreCase("Red Nectar") ||
                    name.equalsIgnoreCase("Red")) {

                return "RED";
            }

            if (name.equalsIgnoreCase("Blue Nectar") ||
                    name.equalsIgnoreCase("Blue")) {

                return "BLUE";
            }
        }

        return null;
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

    private void shooterOn() {

        shooterMotor.setPower(0.75);
        riserMotor.setPower(0.75);
    }

    private void shooterOff() {

        shooterMotor.setPower(0);
        riserMotor.setPower(0);
    }


}
