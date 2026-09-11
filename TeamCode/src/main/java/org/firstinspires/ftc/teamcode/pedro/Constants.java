package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("bl");
        c.backRightName.set("br");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.xPodPort.set(1);
        c.yPodPort.set(0);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(5.059055118110236);
        c.yPodOffset.set(3.1496062992125986);
        c.xPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.yPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.043783453464582);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.32060869637462247);
                Controller secondaryTranslationalForward = Controller.proportional(0.11845634648827848);
                Controller primaryTranslationalLateral = Controller.proportional(0.44574002179127864);
                Controller secondaryTranslationalLateral = Controller.proportional(0.16468902765913837);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013339224816022146));
                c.brake.set(Controller.proportionalFeedforward(0.011338341093618825));

                c.headingFeedback.set(Controller.proportional(4.967847065747849));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04476828428642984, 0.004474488642375099));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06696815695018951, 0.05554247734452525));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001678593160911575, 0.001521121324725272));

                c.maxAchievableForwardVelocity.set(52.53913495547347);
                c.maxAchievableStrafeVelocity.set(40.96546532607475);
                c.naturalForwardDeceleration.set(42.96421602180683);
                c.naturalStrafeDeceleration.set(73.70128222810082);
            }
    );
}