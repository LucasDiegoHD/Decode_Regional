// Ficheiro: subsystems/VisionSubsystem.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// ** AS IMPORTAÇÕES CORRETAS E NATIVAS DO SDK DA FTC **
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult latestResult;
    private final TelemetryManager telemetry;

    public VisionSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        // O nome da Limelight deve corresponder à sua configuração de hardware.
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

    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    /**
     * VERSÃO FINAL: Calcula a Pose 2D do robô usando o objeto Pose3D nativo do SDK da FTC,
     * que é o que a biblioteca da Limelight retorna.
     *
     * @return um Optional<Pose> contendo a pose do robô no sistema de coordenadas da FTC.
     */
    public Optional<Pose> getRobotPoseFromVision() {
        if (!hasTarget()) {
            return Optional.empty();
        }

        // A API retorna um objeto Pose3D do SDK da FTC.
        Pose3D botpose = latestResult.getBotpose();
        if (botpose == null) {
            return Optional.empty();
        }

        // --- TRANSFORMAÇÃO DE COORDENADAS USANDO OS MÉTODOS NATIVOS ---
        // Acesso aos dados através dos métodos do objeto Pose3D do SDK.
        double ftcX = botpose.getPosition().z;
        double ftcY = -botpose.getPosition().x;
        // Obter o Yaw diretamente em radianos é o método mais limpo.
        double ftcHeading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        return Optional.of(new Pose(ftcX, ftcY, ftcHeading));
    }

    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();
    }
}

