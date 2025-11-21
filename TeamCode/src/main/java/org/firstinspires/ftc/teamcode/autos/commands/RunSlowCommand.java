package org.firstinspires.ftc.teamcode.autos.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

/**
 * A command that drives the robot to a specified {@link Pose} using a single Bezier curve.
 * This command is designed for simple, point-to-point movements within an autonomous sequence.
 * It generates a {@link PathChain} consisting of a single {@link BezierLine} from the robot's
 * current pose to the target pose. The heading is linearly interpolated between the start and end poses.
 */
public class RunSlowCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Constructs a new GoToPoseCommand.
     *
     * @param drivetrain The drivetrain subsystem required by this command.
     */
    public RunSlowCommand(@NonNull DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.getFollower().startTeleOpDrive();
        drivetrain.getFollower().setTeleOpDrive(-Constants.AUTONOMOUS_SPEED, 0, 0);
        timer.reset();

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.getFollower().setTeleOpDrive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= Constants.TIMEOUT_SLOW_COMMAND;
    }
}

