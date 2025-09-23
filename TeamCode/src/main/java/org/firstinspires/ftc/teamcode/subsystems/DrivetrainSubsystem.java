// Ficheiro: subsystems/DrivetrainSubsystem.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    private final Follower follower;
    private VisionSubsystem vision; // ATENÇÃO: Dependência do VisionSubsystem

    /**
     * ATUALIZADO: O construtor agora pode receber o VisionSubsystem.
     * Isto será configurado no RobotContainer.
     */
    public DrivetrainSubsystem(HardwareMap hardwareMap, VisionSubsystem vision) {
        follower = Constants.Drivetrain.createFollower(hardwareMap);
    }

    public void setVision(VisionSubsystem vision) {
        this.vision = vision;
    }

    public Follower getFollower() {
        return follower;
    }

    /**
     * ATUALIZADO: O método periodic agora contém a lógica de fusão de sensores.
     */
    @Override
    public void periodic() {
        // 1. Atualiza a odometria do Pinpoint (movimento relativo)
        follower.update();

        // 2. Se o subsistema de visão foi definido, tenta obter uma correção.
        if (vision != null) {
            vision.getRobotPoseFromVision().ifPresent(visionPose -> {
                // Se a visão nos deu uma pose válida, atualizamos a pose do follower.
                // Isto corrige qualquer desvio acumulado pelo Pinpoint.
                follower.setPose(visionPose);
            });
        }
    }
}
