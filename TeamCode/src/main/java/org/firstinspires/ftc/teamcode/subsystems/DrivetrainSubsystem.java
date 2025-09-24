package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.bylazar.telemetry.TelemetryManager;

public class DrivetrainSubsystem extends SubsystemBase {
    private final Follower follower;
    private final TelemetryManager telemetry;

    public DrivetrainSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        follower = Constants.Drivetrain.createFollower(hardwareMap);
    }

    public Follower getFollower() {
        return follower;
    }

    @Override
    public void periodic() {
        follower.update();
        telemetry.addData("Pose X (in)", follower.getPose().getX());
        telemetry.addData("Pose Y (in)", follower.getPose().getY());
        telemetry.addData("Pose Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
    }
}
