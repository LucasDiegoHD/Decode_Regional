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

    public VisionSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    public Optional<Double> getTargetTx() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTx());
        }
        return Optional.empty();
    }

    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    public Optional<Pose> getRobotPoseFromVision() {
        if (!hasTarget()) {
            return Optional.empty();
        }

        Pose3D botpose = latestResult.getBotpose();
        if (botpose == null) {
            return Optional.empty();
        }

        double ftcX = botpose.getPosition().z;
        double ftcY = -botpose.getPosition().x;
        double ftcHeading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        return Optional.of(new Pose(ftcX, ftcY, ftcHeading));
    }

    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();
    }
}

