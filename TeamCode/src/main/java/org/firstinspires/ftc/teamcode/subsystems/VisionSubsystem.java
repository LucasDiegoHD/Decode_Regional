package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult latestResult;
    private final TelemetryManager telemetry;

    public VisionSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start(); // Inicia a coleta de dados da câmera
        limelight.pipelineSwitch(0);
        this.telemetry = telemetry;
    }

    /**
     * Retorna o desvio horizontal (ângulo) do alvo em graus.
     * Valor negativo significa que o alvo está à esquerda, positivo à direita.
     *
     * @return um Optional contendo o valor de 'tx', ou vazio se nenhum alvo for válido.
     */
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
     * Retorna o desvio vertical (ângulo) do alvo em graus.
     * Pode ser usado como um indicador de distância.
     *
     * @return um Optional contendo o valor de 'ty', ou vazio se nenhum alvo for válido.
     */
    public Optional<Double> getTargetTa() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTa());
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
            telemetry.addData("LL ta", latestResult.getTa());
        } else {
            telemetry.addLine("LL sem resultado");
        }
    }
}