package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DrivetrainSubsystem extends SubsystemBase {

    private final Follower follower;
    private final VisionSubsystem vision;
    private final TelemetryManager telemetry;

    public DrivetrainSubsystem(HardwareMap hardwareMap, VisionSubsystem vision, TelemetryManager telemetry) {
        this.vision = vision;
        this.telemetry = telemetry;
        this.follower = Constants.Drivetrain.createFollower(hardwareMap);
    }

    public Follower getFollower() {
        return follower;
    }

    @Override
    public void periodic() {
        follower.update();

        vision.getRobotPoseFromVision().ifPresent(visionPose -> {
            getFollower().setPose(visionPose);
        });

        Pose currentPose = getFollower().getPose();
        if (currentPose != null) {
            telemetry.debug("Robot X (inch): " + currentPose.getX());
            telemetry.debug("Robot Y (inch): " + currentPose.getY());
            telemetry.debug("Robot Heading (deg): " + Math.toDegrees(currentPose.getHeading()));
        }
    }
}

