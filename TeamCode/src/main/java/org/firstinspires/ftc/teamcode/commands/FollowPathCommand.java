package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class FollowPathCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final PathChain path;
    private final TelemetryManager telemetry;

    public FollowPathCommand(DrivetrainSubsystem drivetrain, PathChain path, TelemetryManager telemetry) {
        this.drivetrain = drivetrain;
        this.path = path;
        this.telemetry = telemetry;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.getFollower().followPath(path);
    }

    @Override
    public void execute() {
        // Envia a pose atual do robô para a telemetria enquanto o caminho é seguido
        telemetry.addData("Pose X (in)", drivetrain.getFollower().getPose().getX());
        telemetry.addData("Pose Y (in)", drivetrain.getFollower().getPose().getY());
        telemetry.addData("Pose Heading (deg)", Math.toDegrees(drivetrain.getFollower().getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public boolean isFinished() {
        return !drivetrain.getFollower().isBusy();
    }
}
