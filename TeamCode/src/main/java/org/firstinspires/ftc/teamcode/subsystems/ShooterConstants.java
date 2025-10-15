package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConstants {
    private ShooterConstants() {} // Impede a instanciação
        public static String RSHOOTER_MOTOR_NAME = "rightShooterMotor";
        public static String LSHOOTER_MOTOR_NAME = "leftShooterMotor";
        public static String TRIGGER_MOTOR_NAME = "triggerMotor";
        public static String HOOD_SERVO_NAME = "Hood_Servo";
        public static final double TICKS_PER_REV = 28.0; // Ajuste conforme o motor
        public static double kP = 0.01;
        public static double kI = 0.000;
        public static double kD = 0.00001;
        public static double kF = 0;
        public static double TARGET_VELOCITY = 4300;
        public static double VELOCITY_TOLERANCE = 100;
        public static double MINIMUM_HOOD = 0.25;
        public static double MAXIMUM_HOOD = 1.0;


}