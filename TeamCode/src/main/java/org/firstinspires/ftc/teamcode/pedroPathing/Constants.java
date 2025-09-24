// Ficheiro: pedroPathing/Constants.java
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

    // ... (As suas classes Drivetrain e Shooter permanecem as mesmas) ...
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
                    .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
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
        public static String SHOOTER_MOTOR_NAME = "shooterMotor";
        public static double kP = 0.01;
        public static double kI = 0.0;
        public static double kD = 0.0001;
        public static double kF = 0.05;
        public static double TARGET_VELOCITY = 2200.0;
        public static double VELOCITY_TOLERANCE = 50.0;
    }
    public static Shooter shooter = new Shooter();

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

    /**
     * NOVO: Contém a lógica geométrica do campo de jogo.
     */
    public static class FieldGeometry {
        /**
         * Verifica se o robô está dentro do triângulo de lançamento definido.
         * A lógica é: a posição x do robô deve ser positiva e a sua posição y
         * deve ser menor que a sua posição x.
         * @param robotPose A pose atual do robô.
         * @return true se o robô estiver na zona de lançamento.
         */
        public static boolean isInLaunchTriangle(Pose robotPose) {
            if (robotPose == null) return false;
            // A sua regra: x > 0 e y < x
            return robotPose.getX() > 0 && robotPose.getY() < robotPose.getX();
        }
    }


    public static class FieldPositions {
        public static final Pose START_POSE = new Pose(0, 0, Math.toRadians(-60));
        public static final Pose SHOOTING_POSE = new Pose(12, 60, Math.toRadians(90));
        public static final Pose PARK_POSE = new Pose(10, 120, Math.toRadians(0));
    }
}
