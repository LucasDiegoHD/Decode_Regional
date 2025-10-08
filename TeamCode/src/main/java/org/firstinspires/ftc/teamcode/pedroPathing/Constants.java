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

    public static class Drivetrain {
        public static class Hardware {
            public static String LEFT_FRONT_MOTOR = "leftFront";
            public static String RIGHT_FRONT_MOTOR = "rightFront";
            public static String LEFT_REAR_MOTOR = "leftRear";
            public static String RIGHT_REAR_MOTOR = "rightRear";
            public static String PINPOINT_LOCALIZER = "pinpoint";
        }

        public static class PedroPathing {
            public static FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()
                    .mass(13)
                    .forwardZeroPowerAcceleration(-25.138658560079815)
                    .lateralZeroPowerAcceleration(-78.96531426769552)
                    .translationalPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.05, 0))
                    .headingPIDFCoefficients(new PIDFCoefficients(3, 0.1, 0.2, 0))
                    .drivePIDFCoefficients(
                            new FilteredPIDFCoefficients(0.025, 0, 0.0002, 1, 0)
                    );

            public static MecanumConstants MECANUM_CONSTANTS = new MecanumConstants()
                    .leftFrontMotorName(Hardware.LEFT_FRONT_MOTOR)
                    .leftRearMotorName(Hardware.LEFT_REAR_MOTOR)
                    .rightFrontMotorName(Hardware.RIGHT_FRONT_MOTOR)
                    .rightRearMotorName(Hardware.RIGHT_REAR_MOTOR)
                    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .xVelocity(82.00584290004647)
                    .yVelocity(65.006779349307)
                    .useBrakeModeInTeleOp(true);

            public static PinpointConstants LOCALIZER_CONSTANTS = new PinpointConstants()
                    .forwardPodY(-5.5)
                    .strafePodX(7)
                    .distanceUnit(DistanceUnit.INCH)
                    .hardwareMapName(Hardware.PINPOINT_LOCALIZER)
                    .encoderResolution(
                            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                    );

            public static PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                    0.995, 500, 2.5, 1
            );
        }

        public static Follower createFollower(HardwareMap hardwareMap) {
            return new FollowerBuilder(PedroPathing.FOLLOWER_CONSTANTS, hardwareMap)
                    .mecanumDrivetrain(PedroPathing.MECANUM_CONSTANTS)
                    .pinpointLocalizer(PedroPathing.LOCALIZER_CONSTANTS)
                    .pathConstraints(PedroPathing.PATH_CONSTRAINTS)
                    .build();
        }
    }

    public static class Shooter {
        public static String RSHOOTER_MOTOR_NAME = "rightShooterMotor";
        public static String LSHOOTER_MOTOR_NAME = "leftShooterMotor";
        public static String TRIGGER_MOTOR_NAME = "triggerMotor";
        public static String HOOD_SERVO_NAME = "Hood_Servo";
        public static double kP = 0.05;
        public static double kI = 0.0;
        public static double kD = 0.0001;
        public static double kF = 0.05;
        public static double TARGET_VELOCITY = 2500.0;
        public static double VELOCITY_TOLERANCE = 50.0;

        public static class ShooterHood {
            public static double LOW_SHOT_POSITION = 0.1;
            public static double MID_SHOT_POSITION = 0.55;
            public static double LONG_SHOT_POSITION = 0.6;
            public static double RETRACTED_POSITION = 1.0;
        }
    }
    public static Shooter shooter = new Shooter();
    public static Shooter.ShooterHood shooterHood = new Shooter.ShooterHood();

    public static class Vision {
        public static double TURN_KP = 0.0325;
        public static double TURN_KI = 0.07;
        public static double TURN_KD = 0.0035;
        public static double TURN_KF = 0.3;
    }

    public static Vision vision = new Vision();

    public static class Intake {
        public static String INTAKE_MOTOR = "intakeMotor";
    }

    public static class FieldPositions {
        public static Pose START_POSE = new Pose(10, -60, Math.toRadians(90));
        public static Pose SHOOTING_POSE = new Pose(20, -35, Math.toRadians(90));
        public static Pose PARK_POSE = new Pose(10, 5, Math.toRadians(90));
        public static Pose SCORING_POSITION = new Pose(48, 72, Math.toRadians(90));

        public static boolean isInLaunchTriangle(Pose currentPose) {
            // Implementação de verificação de área de arremesso, se necessário.
            // Por enquanto, retorna true para fins de demonstração.
            return true;
        }
    }
}