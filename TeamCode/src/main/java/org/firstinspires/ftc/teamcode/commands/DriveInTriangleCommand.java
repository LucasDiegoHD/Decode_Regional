package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

/**
 * Um comando que faz o robô dirigir em um caminho triangular.
 */
public class DriveInTriangleCommand extends SequentialCommandGroup {

    public DriveInTriangleCommand(DrivetrainSubsystem drivetrain, TelemetryManager telemetry) {
        Pose startTrianglePose = drivetrain.getFollower().getPose();

        // Define os vértices do triângulo a partir da pose inicial do robô.
        // As coordenadas são em polegadas.
        // NOTA: A classe Pose não tem o método 'add'. As novas poses são criadas
        // com base nos valores da pose inicial.
        Pose point1 = new Pose(startTrianglePose.getX() + 24, startTrianglePose.getY(), startTrianglePose.getHeading());
        Pose point2 = new Pose(startTrianglePose.getX() + 12, startTrianglePose.getY() + 24, startTrianglePose.getHeading());

        PathChain trianglePath = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(startTrianglePose, point1))
                .addPath(new BezierLine(point1, point2))
                .addPath(new BezierLine(point2, startTrianglePose))
                .build();

        // Agenda o comando para seguir o caminho triangular.
        addCommands(new FollowPathCommand(drivetrain, trianglePath, telemetry));
    }
}
