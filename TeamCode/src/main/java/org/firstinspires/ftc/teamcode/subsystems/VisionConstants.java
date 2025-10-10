package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class VisionConstants {
    private VisionConstants() {} // Impede a instanciação

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
            public static double MINIMAL_HOOD = 0.1;
            public static double MAXUMUM_HOOD = 0.55;
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