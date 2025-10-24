package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static class Drivetrain {
            public static String LEFT_FRONT_MOTOR = "leftFront";
            public static String RIGHT_FRONT_MOTOR = "rightFront";
            public static String LEFT_REAR_MOTOR = "leftRear";
            public static String RIGHT_REAR_MOTOR = "rightRear";
            public static String PINPOINT_LOCALIZER = "pinpoint";
        }
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13)
            .forwardZeroPowerAcceleration(-42.33732593025041)
            .lateralZeroPowerAcceleration(-97.42340280933647)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0001, 0.01, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(3, 0.1, 0.2, 0.025))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.03, 0.001, 0.001, 1, 0.15)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName(Drivetrain.LEFT_FRONT_MOTOR)
            .leftRearMotorName(Drivetrain.LEFT_REAR_MOTOR)
            .rightFrontMotorName(Drivetrain.RIGHT_FRONT_MOTOR)
            .rightRearMotorName(Drivetrain.RIGHT_REAR_MOTOR)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(77.11611423342248)
            .yVelocity(59.88718660609929)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.5)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(Drivetrain.PINPOINT_LOCALIZER)
            .yawScalar(1.0)
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            500,
            2.5,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }

}
