package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult latestResult;
    private final TelemetryManager telemetry;

    // Initializes the Limelight camera.
    public VisionSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    // Gets the horizontal angle to the target.
    public Optional<Double> getTargetTx() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTx());
        }
        return Optional.empty();
    }

    // Gets the vertical angle to the target.
    public Optional<Double> getTargetTy() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTy());
        }
        return Optional.empty();
    }

    // Gets the area of the target.
    public Optional<Double> getTargetTa() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTa());
        }
        return Optional.empty();
    }

    // Checks if a valid target is visible.
    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    // Calculates the horizontal (floor) distance to the target.
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

    // Calculates the direct (hypotenuse) distance to the target.
    public Optional<Double> getDirectDistanceToTarget() {
        if (!hasTarget()) {
            return Optional.empty();
        }

        double ty = latestResult.getTy();
        double totalAngleRadians = Math.toRadians(VisionConstants.CAMERA_PITCH_DEGREES + ty);

        if (Math.sin(totalAngleRadians) == 0) {
            return Optional.empty();
        }

        double distance = (VisionConstants.TARGET_HEIGHT_METERS - VisionConstants.CAMERA_HEIGHT_METERS) / Math.sin(totalAngleRadians);

        return Optional.of(Math.abs(distance));
    }

    // Calculates the robot's field pose from the Limelight's botpose data.
    public Optional<Pose> getRobotPoseFromVision(double imuHeadingRadians) {
        if (!hasTarget()) {
            return Optional.empty();
        }
        Pose3D botposeMeters = latestResult.getBotpose();
        if (botposeMeters == null) {
            return Optional.empty();
        }

        double ftcX_meters = botposeMeters.getPosition().z;
        double ftcY_meters = -botposeMeters.getPosition().x;

        return Optional.of(new Pose(ftcX_meters, ftcY_meters, imuHeadingRadians));
    }

    // Updates Limelight data and sends it to telemetry.
    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();

        if (latestResult != null && latestResult.isValid()) {
            telemetry.addData("LL Valid", true);
            telemetry.addData("LL tx", latestResult.getTx());
            telemetry.addData("LL ty", latestResult.getTy());
            telemetry.addData("LL ta", latestResult.getTa());

            // Added back the distance telemetry.
            getHorizontalDistanceToTarget().ifPresent(distance -> {
                telemetry.addData("Distancia Horizontal (M)", distance);
            });
            getDirectDistanceToTarget().ifPresent(distance -> {
                telemetry.addData("Distancia Direta (Hipotenusa)", distance);
            });

            // Added the raw botpose telemetry for debugging.
            Pose3D botpose = latestResult.getBotpose();
            if (botpose != null) {
                telemetry.addData("Botpose X (raw)", botpose.getPosition().x);
                telemetry.addData("Botpose Y (raw)", botpose.getPosition().y);
                telemetry.addData("Botpose Z (raw)", botpose.getPosition().z);
            }

        } else {
            telemetry.addData("LL Valid", false);
            telemetry.addLine("LL sem resultado");
        }
    }
}

