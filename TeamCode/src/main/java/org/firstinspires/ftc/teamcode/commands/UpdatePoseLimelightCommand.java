package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * A command that continuously updates the Limelight camera's internal robot yaw orientation.
 * This helps the Limelight provide a more accurate field-relative pose estimation by combining
 * its vision data with the robot's odometry/IMU heading.
 */
public class UpdatePoseLimelightCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final Pose pose;

    /**
     * Creates a new UpdateLimelightYawCommand.
     *
     * @param drivetrain The DrivetrainSubsystem to get the robot's heading from.
     * @param vision     The VisionSubsystem to update.
     */
    public UpdatePoseLimelightCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, Pose initialPose) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.pose = initialPose;
        addRequirements(vision); // This command modifies the state of the vision subsystem
    }

    /**
     * Called repeatedly while the command is scheduled. Gets the robot's current yaw from the
     * drivetrain and sends it to the vision subsystem.
     */
    @Override
    public void initialize() {


        /*Pose p = vision.getRobotPose();
        if (p != null) {

            drivetrain.getFollower().setPose(p);

        } else {*/
            drivetrain.getFollower().setPose(pose);
        //}

    }

}