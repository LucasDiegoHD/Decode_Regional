package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConstants {
    private ShooterConstants() {} // Impede a instanciação
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

            public static double MINIMUM_HOOD = 0.25;
            public static double MAXIMUM_HOOD = 1.0;


}