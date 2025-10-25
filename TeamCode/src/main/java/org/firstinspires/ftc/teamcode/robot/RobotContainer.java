package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.autos.commands.AutonomousCommands;
import org.firstinspires.ftc.teamcode.autos.paths.BlueRearPoses;
import org.firstinspires.ftc.teamcode.commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.SpinShooterCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveAimingCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.UpdateLimelightYawCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class RobotContainer {

    private final DrivetrainSubsystem drivetrain;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final IndexerSubsystem indexer;




    Pose startPose = new Pose(0, 0, Math.toRadians(0));
    Pose parkPose = new Pose(10, 120, Math.toRadians(0));

    public RobotContainer(HardwareMap hardwareMap, TelemetryManager telemetry, GamepadEx driver, GamepadEx operator) {
        drivetrain = new DrivetrainSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        vision = new VisionSubsystem(hardwareMap, telemetry);
        indexer = new IndexerSubsystem(hardwareMap, telemetry);


        drivetrain.getFollower().setPose(vision.getRobotPose(Math.PI).orElse(new Pose(60,-11,Math.PI)));
        vision.setDefaultCommand(new UpdateLimelightYawCommand(drivetrain,vision));
        if (driver != null) {
            drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, driver));

            new GamepadButton(driver, GamepadKeys.Button.Y)
                    .whileHeld(new AlignToAprilTagCommand(drivetrain, vision, shooter, telemetry,driver));

            new GamepadButton(driver, GamepadKeys.Button.X)
                    .whileHeld(new TeleOpDriveAimingCommand(drivetrain,driver, 1.8796, -1.8796));


        }
        if (operator != null) {
            configureTeleOpBindings(operator, telemetry);

        }
    }

    public Command getAutonomousBlueRearCommand() {
        return new AutonomousCommands(drivetrain, shooter, intake, BlueRearPoses.asList(), true);
    }
    private void configureTeleOpBindings(GamepadEx operator, TelemetryManager telemetry) {

        new GamepadButton(operator, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(shooter::decreaseHood, shooter));

        new GamepadButton(operator, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(shooter::increaseHood, shooter));

        new GamepadButton(operator, GamepadKeys.Button.DPAD_UP)
                .whileHeld(new ShootCommand(shooter, intake));


        new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(shooter::stop, shooter));

        new GamepadButton(operator, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(intake::run, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        new GamepadButton(operator, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(intake::reverse, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        new GamepadButton(operator, GamepadKeys.Button.B)
                .whenPressed(new SpinShooterCommand(shooter, SpinShooterCommand.Action.SHORT_SHOOT));

        new GamepadButton(operator,GamepadKeys.Button.X)
                .whenPressed(new SpinShooterCommand(shooter, SpinShooterCommand.Action.LONG_SHOOT));
    }

}
