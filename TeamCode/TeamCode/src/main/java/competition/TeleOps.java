package competition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config.Intake;
import org.firstinspires.ftc.teamcode.Config.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Config.Shooting;

@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class TeleOps extends OpMode {

    private MecanumDrivebase drivebase;
    private Intake intake;
    private Shooting shooter;

    private boolean previousCircle = false;
    private boolean previousTriangle = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;

    @Override
    public void init() {
        drivebase = new MecanumDrivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooting(hardwareMap);

        telemetry.addLine("CIRCUIT RUNNERS");
        telemetry.addLine("MECANUM TELEOP");
        telemetry.addLine("READY!");
        telemetry.update();
    }

    @Override
    public void loop() {

        double forward =
                -gamepad1.left_stick_y
                        - gamepad2.left_stick_y;

        double right =
                gamepad1.left_stick_x
                        - gamepad2.left_stick_x;

        double rotation =
                -gamepad1.right_stick_x
                        + gamepad2.right_stick_x;

        boolean circle =
                gamepad1.circle || gamepad2.circle;

        if (circle && !previousCircle) {
            drivebase.toggleHeadingHold();
        }

        previousCircle = circle;

        boolean triangle =
                gamepad1.triangle || gamepad2.triangle;

        if (triangle && !previousTriangle) {
            drivebase.toggleThreeYearOldMode();
        }

        previousTriangle = triangle;

        boolean precisionMode =
                gamepad1.square || gamepad2.square;

        boolean dpadUp =
                gamepad1.dpad_up || gamepad2.dpad_up;

        if (dpadUp && !previousDpadUp) {
            drivebase.increasePower();
        }

        previousDpadUp = dpadUp;

        boolean dpadDown =
                gamepad1.dpad_down || gamepad2.dpad_down;

        if (dpadDown && !previousDpadDown) {
            drivebase.decreasePower();
        }

        previousDpadDown = dpadDown;

        double intakePower = Math.max(
                gamepad1.right_trigger,
                gamepad2.right_trigger
        );

        double outtakePower = Math.max(
                gamepad1.left_trigger,
                gamepad2.left_trigger
        );

        if (intakePower > 0.05 && outtakePower <= 0.05) {
            intake.intake(intakePower);
        } else if (outtakePower > 0.05 && intakePower <= 0.05) {
            intake.outtake(outtakePower);
        } else {
            intake.stop();
        }

        boolean boost =
                gamepad1.right_bumper
                        || gamepad2.right_bumper;

        boolean reverseShooter =
                gamepad1.left_bumper
                        || gamepad2.left_bumper;

        if (reverseShooter && !boost) {
            shooter.reverse(1.0);
        } else if (boost) {
            shooter.boost();
        } else {
            shooter.shoot();
        }

        boolean riserForward =
                gamepad1.dpad_left
                        || gamepad2.dpad_left;

        boolean riserReverse =
                gamepad1.dpad_right
                        || gamepad2.dpad_right;

        if (riserForward && !riserReverse) {
            shooter.riser(1.0);
        } else if (riserReverse && !riserForward) {
            shooter.riser(-1.0);
        } else {
            shooter.riser(0);
        }

        shooter.update();

        drivebase.drive(
                forward,
                right,
                rotation,
                precisionMode
        );

        telemetry.addData(
                "Power",
                "%.0f%%",
                drivebase.getMaxPower() * 100
        );

        telemetry.addData(
                "3-Year-Old Mode",
                drivebase.isThreeYearOldMode()
                        ? "ON"
                        : "OFF"
        );

        telemetry.addData(
                "Precision",
                precisionMode ? "ON" : "OFF"
        );

        telemetry.addData(
                "Intake",
                intakePower > 0.05
                        ? "INTAKE"
                        : outtakePower > 0.05
                          ? "OUTTAKE"
                          : "STOPPED"
        );

        telemetry.addData(
                "Shooter RPM",
                "%.0f",
                shooter.getRPM()
        );

        telemetry.addData(
                "Target RPM",
                "%.0f",
                shooter.getTargetRPM()
        );

        telemetry.addData(
                "Shooter Ready",
                shooter.isReady()
                        ? "YES"
                        : "NO"
        );

        telemetry.addData(
                "Shooter",
                reverseShooter
                        ? "REVERSE"
                        : boost
                          ? "BOOST"
                          : "NORMAL"
        );

        telemetry.addData(
                "Riser",
                riserForward
                        ? "FORWARD"
                        : riserReverse
                          ? "REVERSE"
                          : "STOPPED"
        );

        telemetry.update();
    }

    @Override
    public void stop() {
        drivebase.drive(0, 0, 0, false);
        intake.stop();
        shooter.stop();
    }
}