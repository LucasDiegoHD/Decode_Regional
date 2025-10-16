package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class RobotContainer {

    // --- AUTONOMOUS SELECTION ---
    public enum AutonomousRoutine {
        SHOOT_PRELOAD_ONLY,
        DO_NOTHING
    }
    // **SELECT YOUR AUTONOMOUS ROUTINE HERE**
    private final AutonomousRoutine selectedAuto = AutonomousRoutine.SHOOT_PRELOAD_ONLY;

    private final DrivetrainSubsystem drivetrain;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final IMU imu;

    public RobotContainer(HardwareMap hardwareMap, TelemetryManager telemetry, GamepadEx driver, GamepadEx operator) {
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        vision = new VisionSubsystem(hardwareMap, telemetry);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(myIMUparameters);

        drivetrain = new DrivetrainSubsystem(hardwareMap, vision, telemetry, imu);

        if (driver != null) {
            drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, driver, imu));
        }

        configureButtonBindings(driver, operator, telemetry);
    }

    private void configureButtonBindings(GamepadEx driver, GamepadEx operator, TelemetryManager telemetry) {
        if (driver != null && operator != null) {
            new GamepadButton(driver, GamepadKeys.Button.Y)
                    .whileHeld(new AlignToAprilTagCommand(drivetrain, vision, shooter, telemetry, operator));
        }

        if (operator != null) {
            new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(new AutoShootCommand(drivetrain, vision, shooter, intake, telemetry, operator));

            new GamepadButton(operator, GamepadKeys.Button.B)
                    .whenPressed(new ShootCommand(shooter, ShootCommand.Action.SPIN_UP, ShooterConstants.TARGET_VELOCITY));

            new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER)
                    .whenPressed(new InstantCommand(shooter::stop, shooter));

            new GamepadButton(operator, GamepadKeys.Button.DPAD_LEFT)
                    .whenPressed(new InstantCommand(shooter::decreaseHood, shooter));

            new GamepadButton(operator, GamepadKeys.Button.DPAD_RIGHT)
                    .whenPressed(new InstantCommand(shooter::increaseHood, shooter));

            new GamepadButton(operator, GamepadKeys.Button.Y)
                    .whenPressed(new InstantCommand(intake::run, intake))
                    .whenReleased(new InstantCommand(intake::stop, intake));

            new GamepadButton(operator, GamepadKeys.Button.A)
                    .whenPressed(new InstantCommand(intake::reverse, intake))
                    .whenReleased(new InstantCommand(intake::stop, intake));

            new GamepadButton(operator, GamepadKeys.Button.DPAD_UP)
                    .whenPressed(new InstantCommand(intake::runTrigger, intake))
                    .whenReleased(new InstantCommand(intake::stopTrigger, intake));
        }
    }

    /**
     * Use this method to get the autonomous command selected above.
     * @return The selected autonomous command.
     */
    public Command getAutonomousCommand(TelemetryManager telemetry, GamepadEx operator) {
        // Defines the starting position for the autonomous period.
        // IMPORTANT: The robot must be physically placed at this starting pose on the field.
        drivetrain.getFollower().setStartingPose(Constants.FieldPositions.START_POSE);

        switch (selectedAuto) {
            case SHOOT_PRELOAD_ONLY:
                // This routine reuses our "super-command" from TeleOp. It will align,
                // adjust the shooter, wait until it's at speed, and then fire.
                return new AutoShootCommand(drivetrain, vision, shooter, intake, telemetry, operator);

            case DO_NOTHING:
            default:
                // Returns an empty command that does nothing.
                return new InstantCommand();
        }
    }
}

