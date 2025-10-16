package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class VisionConstants {
    private VisionConstants() {}

    public static double TURN_KP = 0.0325;
    public static double TURN_KI = 0.07;
    public static double TURN_KD = 0.0035;
    public static double TURN_KF = 0.3;
    public static double MAXIMUM_TA = 4.3;
    public static double MINIMUM_TA = 0.32;
    public static double CAMERA_HEIGHT_METERS = 0.24;
    public static double TARGET_HEIGHT_METERS = 0.72;
    public static double CAMERA_PITCH_DEGREES = 26.2;
    /**
     * Calculates the required hood angle based on the distance to the target.
     * TODO: This is a placeholder. Replace with your actual calibrated interpolation table or formula.
     * @param distanceMeters The horizontal distance to the target.
     * @return The calculated servo position for the hood (0.0 to 1.0).
     */
    public static double calculateHoodPosition(double distanceMeters) {
        // Placeholder linear formula: assumes hood angle increases with distance.
        // You will replace this with your own calibration data.
        double minDistance = 1.0; // meters
        double maxDistance = 4.0; // meters
        double minHood = ShooterConstants.MINIMUM_HOOD;
        double maxHood = ShooterConstants.MAXIMUM_HOOD;

        if (distanceMeters <= minDistance) return minHood;
        if (distanceMeters >= maxDistance) return maxHood;

        double ratio = (distanceMeters - minDistance) / (maxDistance - minDistance);
        return minHood + ratio * (maxHood - minHood);
    }

    /**
     * Calculates the required shooter RPM based on the distance to the target.
     * TODO: This is a placeholder. Replace with your actual calibrated interpolation table or formula.
     * @param distanceMeters The horizontal distance to the target.
     * @return The calculated target RPM for the shooter.
     */
    public static double calculateShooterRpm(double distanceMeters) {
        // Placeholder linear formula: assumes RPM increases with distance.
        // You will replace this with your own calibration data.
        if (distanceMeters < 1.5) return 4000;
        if (distanceMeters < 3.0) return 4300;
        return 4600;
    }
}
