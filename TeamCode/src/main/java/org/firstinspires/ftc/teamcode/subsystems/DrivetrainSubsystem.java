package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DrivetrainSubsystem extends SubsystemBase {

    private Follower follower;
    private final VisionSubsystem vision;
    private final TelemetryManager telemetry;
    private boolean isInitializedSuccessfully = false;

    public DrivetrainSubsystem(HardwareMap hardwareMap, VisionSubsystem vision, TelemetryManager telemetry) {
        this.vision = vision;
        this.telemetry = telemetry;

        try {
            this.follower = Constants.Drivetrain.createFollower(hardwareMap);
            // garante que a pose inicial não seja nula
            if (this.follower != null) {
                this.follower.update();
                this.follower.setPose(new Pose(0.0, 0.0, 0.0));
            }
            isInitializedSuccessfully = true;
        } catch (Exception e) {
            this.follower = null;
            isInitializedSuccessfully = false;
            if (this.telemetry != null) {
                telemetry.debug("!!! FALHA CRÍTICA AO INICIALIZAR O DRIVETRAINSUBSYSTEM !!!");
                telemetry.debug("VERIFIQUE OS NOMES DOS MOTORES NA CONFIGURAÇÃO DO ROBÔ.");
                telemetry.debug("Erro: " + e.getMessage());
            }
        }
    }

    public Follower getFollower() {
        return follower;
    }

    public boolean isInitializedSuccessfully() {
        return isInitializedSuccessfully && follower != null;
    }

    /**
     * Método seguro para uso em TeleOp: checa follower e pose nula e
     * inicializa uma pose zero se necessário antes de chamar setTeleOpDrive.
     */
    public void teleOpDriveSafe(double forward, double strafe, double turn, boolean fieldCentric) {
        if (!isInitializedSuccessfully() || follower == null) {
            // não crashar: apenas registra que ainda não está pronto
            debug("teleOpDriveSafe: drivetrain não inicializado (ignorando inputs temporariamente).");
            return;
        }

        // Se por algum motivo a pose for nula, inicializa para evitar NPEs internos
        if (follower.getPose() == null) {
            follower.setPose(new Pose(0.0, 0.0, 0.0));
        }

        try {
            follower.setTeleOpDrive(forward, strafe, turn, fieldCentric);
        } catch (Exception e) {
            // segurança: evita crash em runtime por comportamento inesperado do follower
            debug("Erro em teleOpDriveSafe: " + e.getMessage());
        }
    }

    /**
     * Método público simples para enviar mensagens de debug para a telemetria.
     */
    public void debug(String message) {
        if (this.telemetry != null) {
            this.telemetry.debug(message);
        }
    }

    @Override
    public void periodic() {
        if (!isInitializedSuccessfully() || follower == null) {
            return;
        }

        // 1. Atualiza a odometria com os encoders das rodas.
        follower.update();

        // 2. Tenta corrigir a pose com a visão.
        Pose poseBeforeVision = follower.getPose();
        if (poseBeforeVision != null && vision != null) {
            double currentHeading = poseBeforeVision.getHeading();
            vision.getRobotPoseFromVision(currentHeading).ifPresent(visionPose -> {
                follower.setPose(visionPose);
            });
        }

        // 3. Telemetria com a pose final mais atual
        Pose finalPose = follower.getPose();
        if (finalPose != null) {
            debug("Robot X (inch): " + finalPose.getX());
            debug("Robot Y (inch): " + finalPose.getY());
            debug("Robot Heading (deg): " + Math.toDegrees(finalPose.getHeading()));
        }
    }
}
