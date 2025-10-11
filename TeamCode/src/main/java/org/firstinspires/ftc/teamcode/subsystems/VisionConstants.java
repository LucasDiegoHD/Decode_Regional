package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class VisionConstants {
    private VisionConstants() {} // Impede a instanciação
    public static double TURN_KP = 0.0325;
    public static double TURN_KI = 0.07;
    public static double TURN_KD = 0.0035;
    public static double TURN_KF = 0.3;
    public static double MAXIMUM_TA = 4.3;
    public static double MINIMUM_TA = 0.32;

}