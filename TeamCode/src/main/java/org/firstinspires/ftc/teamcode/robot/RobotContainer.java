package org.firstinspires.ftc.teamcode.robot;

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
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class RobotContainer {

    public enum AutonomousRoutine {
        DO_NOTHING,
        SHOOT_THREE,
        DRIVE_AND_SHOOT_AND_PARK
    }
    private final AutonomousRoutine selectedAuto = AutonomousRoutine.DRIVE_AND_SHOOT_AND_PARK;

    // Subsistemas
    private final DrivetrainSubsystem drivetrain;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;

    // Comandos
    private final TeleOpDriveCommand teleOpDriveCommand;

    public RobotContainer(HardwareMap hardwareMap, TelemetryManager telemetry, GamepadEx driver, GamepadEx operator) {
        // A ordem de inicialização dos subsistemas é importante
        this.vision = new VisionSubsystem(hardwareMap, telemetry);
        this.drivetrain = new DrivetrainSubsystem(hardwareMap, vision, telemetry);
        this.intake = new IntakeSubsystem(hardwareMap);
        this.shooter = new ShooterSubsystem(hardwareMap, telemetry);

        this.teleOpDriveCommand = new TeleOpDriveCommand(drivetrain, driver);
        drivetrain.setDefaultCommand(teleOpDriveCommand);

        // Só define como default se o drivetrain foi inicializado corretamente
        if (drivetrain != null && drivetrain.isInitializedSuccessfully()) {
            drivetrain.setDefaultCommand(teleOpDriveCommand);
        } else {
            telemetry.debug("Default command NÃO configurado: drivetrain não inicializado.");
        }

        // Configura todas as ligações de botões
        configureButtonBindings(driver, operator, telemetry);
    }

    private void configureButtonBindings(GamepadEx driver, GamepadEx operator, TelemetryManager telemetry) {
        // --- CONTROLOS DO PILOTO ---
        if (driver != null) {
            new GamepadButton(driver, GamepadKeys.Button.Y)
                    .whileHeld(new AlignToAprilTagCommand(drivetrain, vision, telemetry));

            new GamepadButton(driver, GamepadKeys.Button.X)
                    .whenPressed(new GoToPoseCommand(drivetrain, Constants.FieldPositions.PARK_POSE));

            // BOTÃO PARA ALTERNAR O MODO DE CONDUÇÃO (ex: BACK)
            new GamepadButton(driver, GamepadKeys.Button.BACK)
                    .whenPressed(() -> teleOpDriveCommand.toggleDriveMode());
        }

        // --- CONTROLOS DO OPERADOR ---
        if (operator != null) {
            new GamepadButton(operator, GamepadKeys.Button.B)
                    .whenPressed(new ConditionalCommand(
                            new LaunchSequenceCommand(shooter, intake),
                            new InstantCommand(), // Não faz nada se a condição for falsa
                            () -> Constants.FieldGeometry.isInLaunchTriangle(drivetrain.getFollower().getPose())
                    ));

            new GamepadButton(operator, GamepadKeys.Button.X)
                    .whenPressed(shooter.stopCommand());

            new GamepadButton(operator, GamepadKeys.Button.A)
                    .whenPressed(intake.intakeCommand())
                    .whenReleased(intake.stopCommand());
        }
    }

    public Command getAutonomousCommand() {
        switch (selectedAuto) {
            case SHOOT_THREE:
                return getShootThreeAutoCommand();
            case DRIVE_AND_SHOOT_AND_PARK:
                return getDriveAndShootAndParkAutoCommand();
            case DO_NOTHING:
            default:
                return new InstantCommand();
        }
    }

    // --- FÁBRICAS DE COMANDOS AUTÔNOMOS ---
    private Command getDriveAndShootAndParkAutoCommand() {
        drivetrain.getFollower().setStartingPose(Constants.FieldPositions.START_POSE);

        PathChain driveAndShootPath = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(Constants.FieldPositions.START_POSE, Constants.FieldPositions.SHOOTING_POSE))
                .addParametricCallback(0.7, () -> shooter.spinUpCommand().schedule())
                .build();

        PathChain parkPath = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(Constants.FieldPositions.SHOOTING_POSE, Constants.FieldPositions.PARK_POSE))
                .build();

        return new SequentialCommandGroup(
                new FollowPathCommand(drivetrain, driveAndShootPath),
                new LaunchSequenceCommand(shooter, intake),
                new FollowPathCommand(drivetrain, parkPath)
        );
    }

    public Command getShootThreeAutoCommand() {
        return new SequentialCommandGroup(
                shooter.spinUpCommand(),
                new WaitUntilCommand(() -> shooter.atTargetVelocity(Constants.Shooter.VELOCITY_TOLERANCE))
                        .withTimeout(3000),
                intake.feedCommand(), new WaitCommand(500),
                intake.feedCommand(), new WaitCommand(500),
                intake.feedCommand(), new WaitCommand(250),
                shooter.stopCommand(),
                intake.stopCommand()
        );
    }
}

