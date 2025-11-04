package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autos.commands.AutonomousCommands;
import org.firstinspires.ftc.teamcode.autos.paths.BlueRearPoses;
import org.firstinspires.ftc.teamcode.autos.paths.RedRearPoses;
import org.firstinspires.ftc.teamcode.commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.SpinShooterCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveAimingCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.UpdateLimelightYawCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * This class is the main container for the robot. It holds all subsystems, configures button
 * bindings, and provides commands for autonomous and tele-operated modes.
 * The RobotContainer is the "glue" that connects the robot's hardware and software components.
 */
public class RobotContainer {

    // Subsystem declarations
    private final DrivetrainSubsystem drivetrain;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final IndexerSubsystem indexer;

    /**
     * The constructor for the RobotContainer. It is responsible for initializing all
     * subsystems, setting up default commands, and configuring gamepad button bindings.
     *
     * @param hardwareMap The hardware map from the OpMode, used to initialize hardware components.
     * @param telemetry   The telemetry manager for logging and dashboard output.
     * @param driver      The driver's gamepad (GamepadEx) for controlling the robot's movement.
     * @param operator    The operator's gamepad (GamepadEx) for controlling robot mechanisms.
     */
    public RobotContainer(HardwareMap hardwareMap, TelemetryManager telemetry, GamepadEx driver, GamepadEx operator) {
        drivetrain = new DrivetrainSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        vision = new VisionSubsystem(hardwareMap, telemetry);
        indexer = new IndexerSubsystem(hardwareMap, telemetry);

        // Initialize robot's starting pose, attempting to use Vision first
        drivetrain.getFollower().setPose(vision.getRobotPose(Math.PI).orElse(new Pose(60,-11,Math.PI)));

        // Set default commands
        vision.setDefaultCommand(new UpdateLimelightYawCommand(drivetrain,vision));
        if (driver != null) {
            drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, driver));

            // Driver controller bindings
            new GamepadButton(driver, GamepadKeys.Button.Y)
                    .whileHeld(new AlignToAprilTagCommand(drivetrain, vision, shooter, telemetry,driver));

            new GamepadButton(driver, GamepadKeys.Button.X)
                    .whileHeld(new TeleOpDriveAimingCommand(drivetrain,driver, 1.8796, -1.8796));
        }

        if (operator != null) {
            configureTeleOpBindings(operator, telemetry);
        }
    }

    /**
     * Gets the command group for the Blue Rear autonomous routine.
     *
     * @return A {@link Command} object representing the complete autonomous sequence.
     */
    public Command getAutonomousBlueRearCommand() {
        return new AutonomousCommands(drivetrain, shooter, intake, BlueRearPoses.asList(), true);
    }

    /**
     * Gets the command group for the Red Rear autonomous routine.
     *
     * @return A {@link Command} object representing the complete autonomous sequence.
     */
    public Command getAutonomousRedRearCommand() {
        return new AutonomousCommands(drivetrain, shooter, intake, RedRearPoses.asList(), true);
    }

    /**
     * Configures the button bindings for the operator's gamepad. This method maps specific
     * gamepad buttons to commands that control the robot's mechanisms like the intake and shooter.
     *
     * @param operator  The operator's gamepad (GamepadEx).
     * @param telemetry The telemetry manager, passed to any commands that need it.
     */
    private void configureTeleOpBindings(GamepadEx operator, TelemetryManager telemetry) {
        ShootCommand shoot = new ShootCommand(shooter, intake);

        // Hood controls
        new GamepadButton(operator, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(shooter::decreaseHood, shooter));

        new GamepadButton(operator, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(shooter::increaseHood, shooter));

        // Continuous shooting
        new GamepadButton(operator, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(shoot::schedule))
                .whenReleased(new InstantCommand(shoot::cancel));

        // Stop shooter
        new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(shooter::stop, shooter));

        // Intake controls
        new GamepadButton(operator, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(intake::run, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        new GamepadButton(operator, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(intake::reverse, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        // Spin shooter to preset speeds
        new GamepadButton(operator, GamepadKeys.Button.B)
                .whenPressed(new SpinShooterCommand(shooter, SpinShooterCommand.Action.SHORT_SHOOT));

        new GamepadButton(operator,GamepadKeys.Button.X)
                .whenPressed(new SpinShooterCommand(shooter, SpinShooterCommand.Action.LONG_SHOOT));
    }

}
