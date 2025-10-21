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
    public static double CAMERA_HEIGHT_METERS = 0.24554;
    public static double TARGET_HEIGHT_METERS = 0.72;
    public static double CAMERA_PITCH_DEGREES = 24.44;
}
