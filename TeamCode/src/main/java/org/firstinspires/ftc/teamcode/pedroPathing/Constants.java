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
    private Constants() {} // Impede a instanciação
    public static int teste = 0;
    @Configurable
    public static class Drivetrain {
        public static class Hardware {
            public static String LEFT_FRONT_MOTOR = "leftFront";
            public static String RIGHT_FRONT_MOTOR = "rightFront";
            public static String LEFT_REAR_MOTOR = "leftRear";
            public static String RIGHT_REAR_MOTOR = "rightRear";
            public static String PINPOINT_LOCALIZER = "pinpoint";
        }
        @Configurable
        public static class PedroPathing {
            public static FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()
                    .mass(13)
                    .forwardZeroPowerAcceleration(-42.3925338692562)
                    .lateralZeroPowerAcceleration(-77.71046201171953)
                    .translationalPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.05, 0))
                    .headingPIDFCoefficients(new PIDFCoefficients(1, 0.001, 0.2, 0))
                    .drivePIDFCoefficients(
                            new FilteredPIDFCoefficients(0.025, 0, 0.0002, 1, 0)
                    );

            public static MecanumConstants MECANUM_CONSTANTS = new MecanumConstants()
                    .leftFrontMotorName(Hardware.LEFT_FRONT_MOTOR)
                    .leftRearMotorName(Hardware.LEFT_REAR_MOTOR)
                    .rightFrontMotorName(Hardware.RIGHT_FRONT_MOTOR)
                    .rightRearMotorName(Hardware.RIGHT_REAR_MOTOR)
                    .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .xVelocity(65.7032605268824)
                    .yVelocity(87.5552968037894)
                    .useBrakeModeInTeleOp(true);

            public static PinpointConstants LOCALIZER_CONSTANTS = new PinpointConstants()
                    .forwardPodY(0)
                    .strafePodX(-6.)
                    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                    .distanceUnit(DistanceUnit.INCH)
                    .hardwareMapName(Hardware.PINPOINT_LOCALIZER)
                    .encoderResolution(
                            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                    );

            public static PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                    0.995, 500, 2.5, 1
            );
        }
        public static PedroPathing pedroPathing = new PedroPathing();



        public static Follower createFollower(HardwareMap hardwareMap) {
            return new FollowerBuilder(PedroPathing.FOLLOWER_CONSTANTS, hardwareMap)
                    .mecanumDrivetrain(PedroPathing.MECANUM_CONSTANTS)
                    .pinpointLocalizer(PedroPathing.LOCALIZER_CONSTANTS)
                    .pathConstraints(PedroPathing.PATH_CONSTRAINTS)
                    .build();
        }
    }

    public static class FieldPositions {
        public static Pose START_POSE = new Pose(10, -60, Math.toRadians(90));
        public static Pose SHOOTING_POSE = new Pose(20, -35, Math.toRadians(90));
        public static Pose PARK_POSE = new Pose(10, 5, Math.toRadians(90));
        public static Pose SCORING_POSITION = new Pose(48, 72, Math.toRadians(90));

        public static boolean isInLaunchTriangle(Pose currentPose) {
            return true;
        }
    }
}