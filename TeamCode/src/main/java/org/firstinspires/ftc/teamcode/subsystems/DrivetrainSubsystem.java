package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.bylazar.telemetry.TelemetryManager; 

public class DrivetrainSubsystem extends SubsystemBase {
    private final Follower follower;
    private final VisionSubsystem vision;
    private final TelemetryManager telemetry; // Adiciona o TelemetryManager

    public DrivetrainSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry, VisionSubsystem vision) {
        this.telemetry = telemetry;
        this.vision = vision; // Salva a referência para o subsistema de visão
        follower = Constants.Drivetrain.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();
    }

    public Follower getFollower() {
        return follower;
    }


    @Override
    public void periodic() {
        follower.update();
        // A lógica de correção de pose com a visão pode ser chamada periodicamente
        updatePoseWithVision();
    }

    /**
     * Tenta atualizar a pose do robô usando dados da Limelight.
     * Esta função deve ser chamada periodicamente no seu OpMode.
     */
    public void updatePoseWithVision() {
        // Obtém a pose atual estimada pelo Pedro Pathing
        Pose currentPose = follower.getPose();
        double currentYawDegrees = Math.toDegrees(currentPose.getHeading());

        // Obtém a pose da Limelight (se disponível)
        Pose visionPose = vision.getLimelightPose(currentYawDegrees);

        // Se a pose da visão for válida, use-a para corrigir a pose do robô
        if (visionPose != null) {
            follower.setPose(visionPose);

            // Envia a pose da Limelight para a telemetria
            telemetry.addData("Vision Pose X (in)", visionPose.getX());
            telemetry.addData("Vision Pose Y (in)", visionPose.getY());
            telemetry.addData("Vision Pose Heading (deg)", Math.toDegrees(visionPose.getHeading()));

            // Opcional: Mostra a diferença entre as poses
            telemetry.addData("Pose Difference X", visionPose.getX() - currentPose.getX());
            telemetry.addData("Pose Difference Y", visionPose.getY() - currentPose.getY());
            telemetry.addData("Pose Difference Heading", Math.toDegrees(visionPose.getHeading() - currentPose.getHeading()));
        }

        // Envia a pose do Pedro Pathing para a telemetria (sempre)
        telemetry.addData("Current Pose X (in)", currentPose.getX());
        telemetry.addData("Current Pose Y (in)", currentPose.getY());
        telemetry.addData("Current Pose Heading (deg)", Math.toDegrees(currentPose.getHeading()));
    }
}
