package org.firstinspires.ftc.teamcode.autos.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

/**
 * A command that drives the robot to a specified {@link Pose} using a single Bezier curve.
 * This command is designed for simple, point-to-point movements within an autonomous sequence.
 * It generates a {@link PathChain} consisting of a single {@link BezierLine} from the robot's
 * current pose to the target pose. The heading is linearly interpolated between the start and end poses.
 */
public class LeaveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final PathChain path;


    /**
     * Constructs a new GoToPoseCommand.
     *
     * @param drivetrain The drivetrain subsystem required by this command.
     */
    public LeaveCommand(@NonNull DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        Pose p = drivetrain.getFollower().getPose().plus(new Pose(10, 10, 0));
        this.path = drivetrain.getFollower()
                .pathBuilder()
                .addPath(
                        new BezierLine(drivetrain.getFollower().getPose(), p)
                )
                .setLinearHeadingInterpolation(drivetrain.getFollower().getHeading(), p.getHeading())
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

