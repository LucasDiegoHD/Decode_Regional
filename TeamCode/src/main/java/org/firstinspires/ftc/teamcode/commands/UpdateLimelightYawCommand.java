package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.Timer;

/**
 * A command that continuously updates the Limelight camera's internal robot yaw orientation.
 * This helps the Limelight provide a more accurate field-relative pose estimation by combining
 * its vision data with the robot's odometry/IMU heading.
 */
public class UpdateLimelightYawCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final ElapsedTime timeoutPose = new ElapsedTime();

    /**
     * Creates a new UpdateLimelightYawCommand.
     *
     * @param drivetrain The DrivetrainSubsystem to get the robot's heading from.
     * @param vision     The VisionSubsystem to update.
     */
    public UpdateLimelightYawCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(vision); // This command modifies the state of the vision subsystem

    }

    /**
     * Called repeatedly while the command is scheduled. Gets the robot's current yaw from the
     * drivetrain and sends it to the vision subsystem.
     */
    @Override
    public void execute() {
        double yaw = drivetrain.getFollower().getHeading();
        vision.updateLimelightYaw(yaw);
        if (timeoutPose.milliseconds() > VisionConstants.UPDATE_POSE_VISION_TIMEOUT) {
        // The commented-out code below could be used to have the Limelight's pose
        // estimation override the follower's pose, effectively re-localizing the robot.
            Pose p = drivetrain.getFollower().getPose();
            drivetrain.getFollower().setPose(vision.getRobotPose(yaw).orElse(p));
            timeoutPose.reset();
        }

    }

}