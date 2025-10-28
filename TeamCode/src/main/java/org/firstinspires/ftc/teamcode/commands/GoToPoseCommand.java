// Ficheiro: commands/drivetrain/GoToPoseCommand.java
package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class GoToPoseCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final Pose targetPose;

    public GoToPoseCommand(DrivetrainSubsystem drivetrain, Pose targetPose) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose startPose = drivetrain.getFollower().getPose();

        PathChain pathToTarget = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();

        drivetrain.getFollower().followPath(pathToTarget);
    }

    @Override
    public boolean isFinished() {
        return !drivetrain.getFollower().isBusy();
    }
}