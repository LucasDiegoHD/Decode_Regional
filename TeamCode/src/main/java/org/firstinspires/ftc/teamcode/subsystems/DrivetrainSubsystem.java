package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    private final Follower follower;
    private final VisionSubsystem vision;
    private final TelemetryManager telemetry;
    private final IMU imu;

    public DrivetrainSubsystem(HardwareMap hardwareMap, VisionSubsystem vision, TelemetryManager telemetry, IMU imu) {
        this.follower = Constants.Drivetrain.createFollower(hardwareMap);
        this.vision = vision;
        this.telemetry = telemetry;
        this.imu = imu;
    }

    public Follower getFollower() {
        return follower;
    }

    /**
     * This periodic method is the heart of the robot's state management.
     * It's responsible for sensor fusion: combining odometry, IMU, and vision.
     */
    @Override
    public void periodic() {
        // First, update the wheel odometry from Pedro Pathing.
        follower.update();

        // Get the most reliable heading from the IMU.
        double currentHeadingRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Attempt to get a position estimate from the Limelight.
        // The '.ifPresent()' ensures this logic only runs when a valid AprilTag is seen.
        vision.getRobotPoseFromVision(currentHeadingRadians).ifPresent(visionPose -> {
            // This is the core of our sensor fusion. We overwrite the robot's current pose
            // with the absolute position from the camera. This corrects any accumulated drift.
            getFollower().setPose(visionPose);
        });

        // Send final, fused odometry data to telemetry for debugging.
        Pose currentPose = getFollower().getPose();
        if (currentPose != null) {
            telemetry.addData("Odometry X (m)", currentPose.getX());
            telemetry.addData("Odometry Y (m)", currentPose.getY());
            telemetry.addData("Odometry H (deg)", Math.toDegrees(currentPose.getHeading()));
        }
    }
}

