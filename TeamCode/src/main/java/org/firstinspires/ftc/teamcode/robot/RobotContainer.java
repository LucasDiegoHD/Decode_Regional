package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FieldPositions.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class RobotContainer {

    private final DrivetrainSubsystem drivetrain;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;

    public RobotContainer(HardwareMap hardwareMap, TelemetryManager telemetry, GamepadEx driver, GamepadEx operator) {
        this.vision = new VisionSubsystem(hardwareMap, telemetry);
        this.intake = new IntakeSubsystem(hardwareMap);
        this.shooter = new ShooterSubsystem(hardwareMap, telemetry);
        this.drivetrain = new DrivetrainSubsystem(hardwareMap, vision, telemetry);

        drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, driver));

        configureButtonBindings(driver, operator, telemetry);
    }

    private void configureButtonBindings(GamepadEx driver, GamepadEx operator, TelemetryManager telemetry) {
        if (driver != null) {
            new GamepadButton(driver, GamepadKeys.Button.Y)
                    .whileHeld(new AlignToAprilTagCommand(drivetrain, vision, telemetry));

            new GamepadButton(driver, GamepadKeys.Button.X)
                    .whenPressed(new GoToPoseCommand(drivetrain, PARK_POSE));
        }

        if (operator != null) {
            new GamepadButton(operator, GamepadKeys.Button.B)
                    .whenPressed(new ConditionalCommand(
                            shooter.spinUpCommand(),
                            new InstantCommand(),
                            () -> Constants.FieldGeometry.isInLaunchTriangle(drivetrain.getFollower().getPose())
                    ));

            new GamepadButton(operator, GamepadKeys.Button.X)
                    .whenPressed(shooter.stopCommand());
        }
    }

    public Command getAutoAvancadoCommand() {
        drivetrain.getFollower().setStartingPose(START_POSE);

        PathChain driveAndShootPath = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOTING_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOTING_POSE.getHeading())
                .addParametricCallback(0.7, () -> shooter.spinUpCommand().schedule())
                .build();

        PathChain parkPath = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(SHOOTING_POSE, PARK_POSE))
                .setLinearHeadingInterpolation(SHOOTING_POSE.getHeading(), PARK_POSE.getHeading())
                .build();

        return new SequentialCommandGroup(
                new FollowPathCommand(drivetrain, driveAndShootPath),
                new WaitUntilCommand(() -> shooter.atTargetVelocity(Constants.Shooter.VELOCITY_TOLERANCE))
                        .withTimeout(2000),
                new InstantCommand(() -> {
                    System.out.println("Peça sendo atirada!");
                }, intake),
                new WaitCommand(250),
                shooter.stopCommand(),
                new FollowPathCommand(drivetrain, parkPath)
        );
    }

    public Command getShootThreeAutoCommand() {
        return new SequentialCommandGroup(
                shooter.spinUpCommand(),
                new WaitUntilCommand(() -> shooter.atTargetVelocity(Constants.Shooter.VELOCITY_TOLERANCE))
                        .withTimeout(3000),
                new InstantCommand(() -> {}, intake, shooter),
                new WaitCommand(500),
                new InstantCommand(() -> {}, intake, shooter),
                new WaitCommand(500),
                new InstantCommand(() -> {}, intake, shooter),
                new WaitCommand(250),
                shooter.stopCommand()
        );
    }
}

