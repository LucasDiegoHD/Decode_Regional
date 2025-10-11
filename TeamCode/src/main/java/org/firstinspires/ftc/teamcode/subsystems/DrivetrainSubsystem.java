package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    private final Follower follower;
    public DrivetrainSubsystem(HardwareMap hardwareMap) {
        follower = Constants.Drivetrain.createFollower(hardwareMap);
    }


    public Follower getFollower() {
        return follower;
    }

    @Override
    public void periodic() {

        follower.update();

    }
}