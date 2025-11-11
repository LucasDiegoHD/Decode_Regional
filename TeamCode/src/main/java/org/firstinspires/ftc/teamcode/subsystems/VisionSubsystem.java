package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Optional;


/**
 * The VisionSubsystem is responsible for handling all vision-related tasks,
 * including target detection and pose estimation using a Limelight camera.
 */
//@AutoLog
public class VisionSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult latestResult;
    private final TelemetryManager telemetry;

    private static final double INCHES_IN_METER = 39.3701;

    /**
     * Constructs a new VisionSubsystem.
     *
     * @param hardwareMap The hardware map to retrieve hardware devices from.
     * @param telemetry   The telemetry manager for logging.
     */
    public VisionSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    /**
     * Gets the horizontal offset from the crosshair to the target.
     * @return An Optional containing the 'tx' value if a target is present.
     */
    public Optional<Double> getTargetTx() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTx());
        }
        return Optional.empty();
    }

    /**
     * Gets the vertical offset from the crosshair to the target.
     * @return An Optional containing the 'ty' value if a target is present.
     */
    public Optional<Double> getTargetTy() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTy());
        }
        return Optional.empty();
    }

    /**
     * Gets the area of the target in the frame.
     * @return An Optional containing the 'ta' value (target area) if a target is present.
     */
    public Optional<Double> getTargetTa() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTa());
        }
        return Optional.empty();
    }

    /**
     * Checks if the Limelight has a valid target.
     * @return True if a valid target is detected, false otherwise.
     */
    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    /**
     * Calculates the horizontal distance (on the ground) to the target.
     * This is ideal for most interpolations.
     * @return An {@code Optional<Double>} containing the distance in meters.
     */
    public Optional<Double> getHorizontalDistanceToTarget() {
        if (!hasTarget()) {
            return Optional.empty();
        }

        double ty = latestResult.getTy();
        double totalAngleRadians = Math.toRadians(VisionConstants.CAMERA_PITCH_DEGREES + ty);

        if (Math.tan(totalAngleRadians) == 0) {
            return Optional.empty();
        }

        double distance = (VisionConstants.TARGET_HEIGHT_METERS - VisionConstants.CAMERA_HEIGHT_METERS) / Math.tan(totalAngleRadians);

        return Optional.of(distance);
    }

    /**
     * Calculates the direct distance (hypotenuse) from the camera to the target.
     * @return An {@code Optional<Double>} containing the distance in meters.
     */
    public Optional<Double> getDirectDistanceToTarget() {
        if (!hasTarget()) {
            return Optional.empty();
        }

        if(latestResult.getFiducialResults().get(0) != null) {
            // The z-position in camera space is the direct distance to the target
            return Optional.of(Math.abs(latestResult.getFiducialResults().get(0).getTargetPoseCameraSpace().getPosition().z));
        }

        return Optional.empty();

    }

    /**
     * Updates the robot's orientation (yaw) in the Limelight.
     * This helps with more accurate pose estimation.
     * @param yaw The robot's current yaw in radians.
     */
    public void updateLimelightYaw(double yaw){
        limelight.updateRobotOrientation(Math.toDegrees(yaw));
    }

    /**
     * Gets the robot's pose as estimated by the Limelight using AprilTags.
     * @param yaw The robot's current yaw in radians, used to improve the estimate.
     * @return An Optional containing the robot's Pose if a target is visible.
     */
    public Pose getRobotPose(double yaw) {
        limelight.updateRobotOrientation(Math.toDegrees(yaw) - 90);
        latestResult = limelight.getLatestResult();

        if(!hasTarget()){
            return null;
        }
        Pose3D robotPose = latestResult.getBotpose_MT2(); // Using MegaTag2 for potentially better accuracy
        if(robotPose == null){
            return null;
        }
        // Convert from meters (Limelight standard) to inches (PedroPathing standard)
        return new Pose(robotPose.getPosition().x * INCHES_IN_METER, robotPose.getPosition().y * INCHES_IN_METER, Math.toRadians(robotPose.getOrientation().getYaw()), FTCCoordinates.INSTANCE);
    }

    /**
     * This method is called periodically to update the subsystem's state and telemetry.
     */
    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();

        if (latestResult != null) {
            telemetry.addData("LL Valid", latestResult.isValid());
            telemetry.addData("LL Pose MT2", latestResult.getBotpose_MT2());
            getHorizontalDistanceToTarget().ifPresent(distance -> telemetry.addData("Distância Horizontal (M)", distance));
            getDirectDistanceToTarget().ifPresent(distance -> telemetry.addData("Distância Direta (M)", distance));

        } else {
            telemetry.addLine("LL sem resultado");
        }
    }
}
