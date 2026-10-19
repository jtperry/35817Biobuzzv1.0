package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.xPodPort.set(1);
        c.yPodPort.set(0);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(-3.14);
        c.yPodOffset.set(-3.72);
        c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.021190333863466);
    });
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("rf");
        c.frontRightName.set("lf");
        c.backLeftName.set("rr");
        c.backRightName.set("lr");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2625545195944956);
                Controller secondaryTranslationalForward = Controller.proportional(0.09700687940419468);
                Controller primaryTranslationalLateral = Controller.proportional(0.32428003222330465);
                Controller secondaryTranslationalLateral = Controller.proportional(0.11981280698446592);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012070604744266737));
                c.brake.set(Controller.proportionalFeedforward(0.010260014032626726));

                c.headingFeedback.set(Controller.proportional(3.8330266802507924));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04207862843183693, 0.0037771106358589334));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0816396030460744, 0.059026905586686475));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001269610873932346, 0.0014883442411688695));

                c.maxAchievableForwardVelocity.set(83.99482610982179);
                c.maxAchievableStrafeVelocity.set(67.03791712453315);
                c.naturalForwardDeceleration.set(35.11887294284997);
                c.naturalStrafeDeceleration.set(62.09565526949194);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new OctoQuadLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}