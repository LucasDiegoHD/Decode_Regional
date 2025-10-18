package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult latestResult;
    private final TelemetryManager telemetry;

    private static final double INCHES_IN_METER = 39.3701;

    public VisionSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    public Optional<Double> getTargetTx() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTx());
        }
        return Optional.empty();
    }

    public Optional<Double> getTargetTy() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTy());
        }
        return Optional.empty();
    }

    public Optional<Double> getTargetTa() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTa());
        }
        return Optional.empty();
    }

    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    /**
     * Calcula a distância HORIZONTAL (no chão) até o alvo.
     * Ideal para a maioria das interpolações.
     * @return um Optional<Double> contendo a distância em polegadas.
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
     * Calcula a distância DIRETA (hipotenusa) da câmara até o alvo.
     * @return um Optional<Double> contendo a distância em polegadas.
     */
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

    public Optional<Pose> getRobotPose(double yaw) {
        limelight.updateRobotOrientation(Math.toDegrees(yaw));
        latestResult = limelight.getLatestResult();
        if(!hasTarget()){
            return Optional.empty();
        }
        Pose3D robotPose = latestResult.getBotpose_MT2();
        if(robotPose == null){
            return Optional.empty();
        }
        return Optional.of(new Pose(robotPose.getPosition().x * INCHES_IN_METER, robotPose.getPosition().y* INCHES_IN_METER, Math.toRadians(robotPose.getOrientation().getYaw())));
    }
    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();

        if (latestResult != null) {
            telemetry.addData("LL Valid", latestResult.isValid());
            telemetry.addData("LL tx", latestResult.getTx());
            telemetry.addData("LL ty", latestResult.getTy());
            telemetry.addData("LL ta", latestResult.getTa());

            getHorizontalDistanceToTarget().ifPresent(distance -> {
                telemetry.addData("Distância Horizontal (M)", distance);
            });
            getDirectDistanceToTarget().ifPresent(distance -> {
                telemetry.addData("Distância Direta (Hipotenusa)", distance);
            });

        } else {
            telemetry.addLine("LL sem resultado");
        }
    }
}

