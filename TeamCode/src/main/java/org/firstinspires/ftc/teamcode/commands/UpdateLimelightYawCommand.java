package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * A command that continuously updates the Limelight camera's internal robot yaw orientation.
 * This helps the Limelight provide a more accurate field-relative pose estimation by combining
 * its vision data with the robot's odometry/IMU heading.
 */
public class UpdateLimelightYawCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final ElapsedTime timeoutPose = new ElapsedTime();
    private final TelemetryManager telemetryM;
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
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    /**
     * Called repeatedly while the command is scheduled. Gets the robot's current yaw from the
     * drivetrain and sends it to the vision subsystem.
     */
    @Override
    public void execute() {


        double yaw = drivetrain.getFollower().getHeading();
        Pose p = vision.getRobotPose(yaw);
        /*if (timeoutPose.milliseconds() > VisionConstants.UPDATE_POSE_VISION_TIMEOUT && p != null) {

            drivetrain.getFollower().setPose(p);
            timeoutPose.reset();
            telemetryM.addData("Pose Updated", p);
        }*/

    }

}