package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

/**
 * A command to make the robot follow a predefined PathChain.
 */
public class FollowPathCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final PathChain path;

    /**
     * Creates a new FollowPathCommand.
     *
     * @param drivetrain The DrivetrainSubsystem to use for movement.
     * @param path       The PathChain for the robot to follow.
     */
    public FollowPathCommand(DrivetrainSubsystem drivetrain, PathChain path) {
        this.drivetrain = drivetrain;
        this.path = path;
        addRequirements(drivetrain);
    }

    /**
     * Called when the command is initially scheduled. Starts following the path.
     */
    @Override
    public void initialize() {
        drivetrain.getFollower().followPath(path);
    }

    /**
     * Returns true when the command should end.
     *
     * @return True when the robot is no longer busy following the path.
     */
    @Override
    public boolean isFinished() {
        return !drivetrain.getFollower().isBusy();
    }
}