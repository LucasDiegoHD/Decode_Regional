package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Constants for the ShooterSubsystem.
 */
@Configurable
public class ShooterConstants {
    /**
     * The hardware map name for the right shooter motor.
     */
        public static String RSHOOTER_MOTOR_NAME = "rightShooterMotor";
    /** The hardware map name for the left shooter motor. */
        public static String LSHOOTER_MOTOR_NAME = "leftShooterMotor";
    /** The hardware map name for the trigger motor. */
        public static String TRIGGER_MOTOR_NAME = "triggerMotor";
    /** The hardware map name for the hood servo. */
        public static String HOOD_SERVO_NAME = "Hood_Servo";
    /**
     * The number of encoder ticks per revolution for the shooter motors.
     */
    public static final double TICKS_PER_REV = 28.0; // Adjust based on the motor
    /** The proportional gain for the shooter velocity controller. */
        public static double kP = 0.01;
    /** The integral gain for the shooter velocity controller. */
        public static double kI = 0.000;
    /** The derivative gain for the shooter velocity controller. */
        public static double kD = 0.00001;
    /** The feedforward gain for the shooter velocity controller. */
        public static double kF = 0;
    /** The target velocity in RPM for short shots. */
        public static double TARGET_VELOCITY_SHORT = 4420;
    /** The target velocity in RPM for long shots. */
        public static double TARGET_VELOCITY_LONG = 5200;
    /** The acceptable error margin for the shooter's target velocity in RPM. */
        public static double VELOCITY_TOLERANCE = 200;
    /** The minimum position of the hood servo. */
        public static double MINIMUM_HOOD = 0.25;
    /** The maximum position of the hood servo. */
        public static double MAXIMUM_HOOD = 0.9;
    /** The amount to increment or decrement the hood position. */
        public static double HOOD_INCREMENT = 0.02;
    /** The time in milliseconds to wait after intaking before shooting. */
        public static double INTAKE_TIMER_TO_SHOOT = 500;
    /**
     * The time in milliseconds to wait after triggering before shooting.
     */
    public static double TRIGGER_TIMER_TO_SHOOT = 1000;


}