package org.firstinspires.ftc.teamcode.autos.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class GoToPoseCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final PathChain path;


    public GoToPoseCommand(DrivetrainSubsystem drivetrain, Pose poseToGo) {
        this.drivetrain = drivetrain;
        this.path = drivetrain.getFollower()
                .pathBuilder()
                .addPath(
                        new BezierLine(drivetrain.getFollower().getPose(), poseToGo)
                )
                .setLinearHeadingInterpolation(drivetrain.getFollower().getHeading(), poseToGo.getHeading())
                .build();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.getFollower().followPath(path);
    }

    @Override
    public boolean isFinished() {
        return !drivetrain.getFollower().isBusy();
    }
}

