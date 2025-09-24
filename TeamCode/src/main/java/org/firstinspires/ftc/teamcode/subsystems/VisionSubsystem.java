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

    private Limelight3A limelight;
    private LLResult latestResult;
    private final TelemetryManager telemetry;

    public VisionSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        // Tenta obter a Limelight do Hardware Map. Se não estiver lá, o objeto será null.
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
            limelight.pipelineSwitch(0);
        } catch (IllegalArgumentException e) {
            telemetry.addData("VisionSubsystem Error", "Limelight 'limelight' não encontrada no Hardware Map. Visão desativada.");
            telemetry.update();
            limelight = null; // Garante que o objeto é nulo se a busca falhar
        }
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

    /**
     * Sincroniza a orientação do robô com a Limelight para melhorar a precisão.
     * @param headingInDegrees A orientação atual do robô, em graus.
     */
    public void updateRobotOrientation(double headingInDegrees) {
        if (limelight != null) {
            limelight.updateRobotOrientation(headingInDegrees);
        }
    }

    /**
     * Calcula a Pose 2D do robô usando o objeto Pose3D nativo do SDK da FTC.
     * @param imuHeading O Heading atual do robô em radianos, obtido do odometria.
     * @return um Optional<Pose> contendo a pose do robô.
     */
    public Optional<Pose> getRobotPoseFromVision(double imuHeading) {
        if (!hasTarget()) {
            return Optional.empty();
        }

        Pose3D botpose = latestResult.getBotpose();
        if (botpose == null) {
            return Optional.empty();
        }

        double ftcX = botpose.getPosition().z;
        double ftcY = -botpose.getPosition().x;

        // O Pedro Pathing usa polegadas, mas a Pose do SDK da FTC usa milímetros.
        double xInches = ftcX ;
        double yInches = ftcY ;

        return Optional.of(new Pose(xInches, yInches, imuHeading));
    }

    /**
     * Retorna a pose 3D nativa da Limelight, se houver um alvo.
     * Use este método para telemetria ou debug.
     * @return Um Optional<Pose3D> que pode conter a pose do bot.
     */
    public Optional<Pose3D> getBotpose() {
        if (hasTarget()) {
            return Optional.ofNullable(latestResult.getBotpose());
        }
        return Optional.empty();
    }

    @Override
    public void periodic() {
        if (limelight != null) {
            latestResult = limelight.getLatestResult();
        }
    }
}
