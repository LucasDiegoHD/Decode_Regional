package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class TeleOpDriveCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final GamepadEx driver;
    private boolean wasInZone = false;

    public TeleOpDriveCommand(DrivetrainSubsystem drivetrain, GamepadEx driver) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.getFollower().setTeleOpDrive(
                -driver.getLeftY(),
                driver.getLeftX(),
                -driver.getRightX(),
                true
        );

        Pose currentPose = drivetrain.getFollower().getPose();
        boolean isInZone = Constants.FieldGeometry.isInLaunchTriangle(currentPose);

        if (isInZone && !wasInZone) {
            driver.gamepad.rumble(500);
        }
        wasInZone = isInZone;
    }
}

