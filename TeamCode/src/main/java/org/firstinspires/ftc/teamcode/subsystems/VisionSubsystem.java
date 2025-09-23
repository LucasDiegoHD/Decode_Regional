package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final Limelight3A limelight;
    private final TelemetryManager telemetry;
    private LLResult latestResult;

    public VisionSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.addData("VisionSubsystem", "Inicializado!");
        telemetry.update();
    }

    /**
     * Tenta obter e converter a pose mais recente da Limelight.
     *
     * @param currentYawDegrees O ângulo de guinada (heading) atual do robô em graus.
     * @return A pose (x, y, heading) em polegadas, ou null se a pose não for válida.
     */
    public Pose getLimelightPose(double currentYawDegrees) {
        // Atualiza a orientação da Limelight com o yaw do robô para melhor precisão
        limelight.updateRobotOrientation(currentYawDegrees);
        LLResult result = limelight.getLatestResult();
        Pose3D botpose = null;

        // Verifica se a Limelight tem uma pose válida
        if (result != null && result.isValid()) {
            botpose = result.getBotpose_MT2();
            if (botpose != null) {
                // Converte a Pose3D da Limelight (metros) para a Pose 2D do Pedro Pathing (polegadas)
                // 1 polegada = 0.0254 metros
                return new Pose(
                        botpose.getPosition().x / 0.0254,
                        botpose.getPosition().y / 0.0254,
                        Math.toRadians(currentYawDegrees)
                );
            }
        }
        return null;
    }
    public Optional<Double> getTargetTx() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTx());
        }
        return Optional.empty();
    }

    /**
     * Retorna o desvio vertical (ângulo) do alvo em graus.
     * Pode ser usado como um indicador de distância.
     *
     * @return um Optional contendo o valor de 'ty', ou vazio se nenhum alvo for válido.
     */
    public Optional<Double> getTargetTy() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTy());
        }
        return Optional.empty();
    }

    /**
     * Verifica se a Limelight tem um alvo válido.
     *
     * @return true se um alvo válido for detectado, false caso contrário.
     */
    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();
        if (latestResult != null) {
            telemetry.addData("LL Valid", latestResult.isValid());
            telemetry.addData("LL tx", latestResult.getTx());
            telemetry.addData("LL ty", latestResult.getTy());
        } else {
            telemetry.addLine("LL sem resultado");
        }
    }
}
