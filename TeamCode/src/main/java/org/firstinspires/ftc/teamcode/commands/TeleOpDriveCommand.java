package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class TeleOpDriveCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final GamepadEx driverGamepad;

    public TeleOpDriveCommand(DrivetrainSubsystem drivetrain, GamepadEx driverGamepad) {
        this.drivetrain = drivetrain;
        this.driverGamepad = driverGamepad;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.getFollower().startTeleopDrive();
    }

    @Override
    public void execute() {
        // O método foi renomeado para setTeleOpMovementVectors na v2.0
        drivetrain.getFollower().setTeleOpDrive(
                -driverGamepad.getLeftY(),
                driverGamepad.getLeftX(),
                -driverGamepad.getRightX(),
                false
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}