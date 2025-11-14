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
    /**
     * The target velocity in RPM for short shots.
     */
        public static double TARGET_VELOCITY_SHORT = 4420;
    /** The target velocity in RPM for long shots. */
    public static double TARGET_VELOCITY_LONG = 6000;
    /** The acceptable error margin for the shooter's target velocity in RPM. */
        public static double VELOCITY_TOLERANCE = 250;
    /** The minimum position of the hood servo. */
        public static double MINIMUM_HOOD = 0.25;
    /** The maximum position of the hood servo. */
        public static double MAXIMUM_HOOD = 0.9;
    /** The amount to increment or decrement the hood position. */
        public static double HOOD_INCREMENT = 0.02;
    public static double INTAKE_TIME_TO_SHOOT = 800;
    /**
     * The time in milliseconds to wait after triggering before shooting.
     */
    public static double TRIGGER_TIMER_TO_SHOOT = 1000;
    /**
     * Equations for hood
     * y= ângulo do hood
     * x= distância dada pela limelight
     * y= 0.2799x^3-1.7289x^2+3.7225x-2.1911
     */
    public static double HOOD_N0 = -2.1011;
    public static double HOOD_N1 = 3.7225;
    public static double HOOD_N2 = -1.7289;
    public static double HOOD_N3 = 0.2799;
    /**
     * Equations for RPM
     * <p>
     * y= RPM
     * x= distância dada pela limelight
     * <p>
     * y=842.9828x+2790.7711
     */
    public static double RPM_N0 = 2790.7711;
    public static double RPM_N1 = 842.9828;

    public static double ANGLE_KP = 0.3;
    public static double ANGLE_KI = 0.0;
    public static double ANGLE_KD = 0.0;


}