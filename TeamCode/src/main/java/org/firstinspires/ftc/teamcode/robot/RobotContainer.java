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
import com.pedropathing.follower.Follower;
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

    // Enum para selecionar o autônomo. Pode ser ligado a um dashboard no futuro.
    public enum AutonomousRoutine {
        DO_NOTHING,
        SHOOT_THREE,
        DRIVE_AND_SHOOT_AND_PARK
    }
    // **SELECIONE O SEU AUTÔNOMO AQUI**
    private final AutonomousRoutine selectedAuto = AutonomousRoutine.DRIVE_AND_SHOOT_AND_PARK;

    // Subsistemas
    private final DrivetrainSubsystem drivetrain;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;

    public RobotContainer(HardwareMap hardwareMap, TelemetryManager telemetry, GamepadEx driver, GamepadEx operator) {

        this.vision = new VisionSubsystem(hardwareMap, telemetry);
        this.drivetrain = new DrivetrainSubsystem(hardwareMap, telemetry);
        this.intake = new IntakeSubsystem(hardwareMap);
        this.shooter = new ShooterSubsystem(hardwareMap, telemetry);

        // Define o comando padrão para a transmissão (controlo do piloto)
        drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, vision, driver, telemetry));

        // Configura as ligações dos botões do gamepad
        configureButtonBindings(driver, operator, telemetry);
    }

    private void configureButtonBindings(GamepadEx driver, GamepadEx operator, TelemetryManager telemetry) {
        // --- DRIVER CONTROLS ---
        if (driver != null) {
            new GamepadButton(driver, GamepadKeys.Button.Y)
                    .whileHeld(new AlignToAprilTagCommand(drivetrain, vision, telemetry));

            new GamepadButton(driver, GamepadKeys.Button.X)
                    .whenPressed(new GoToPoseCommand(drivetrain, Constants.FieldPositions.PARK_POSE));
        }

        // --- OPERATOR CONTROLS ---
        if (operator != null) {
            new GamepadButton(operator, GamepadKeys.Button.B)
                    .whenPressed(new ConditionalCommand(
                            // Comando a executar se a condição for verdadeira
                            new LaunchSequenceCommand(shooter, intake),
                            // Comando a executar se a condição for falsa
                            new InstantCommand(), // Faz nada
                            // A condição a ser verificada
                            () -> Constants.FieldGeometry.isInLaunchTriangle(drivetrain.getFollower().getPose())
                    ));

            new GamepadButton(operator, GamepadKeys.Button.X)
                    .whenPressed(shooter.stopCommand());

            new GamepadButton(operator, GamepadKeys.Button.A)
                    .whenPressed(intake.intakeCommand())
                    .whenReleased(intake.stopCommand());
        }
    }

    /**
     * Usa o enum selectedAuto para construir e retornar a sequência de comandos do autônomo desejado.
     * @return O comando autônomo selecionado.
     */
    public Command getAutonomousCommand() {
        switch (selectedAuto) {
            case SHOOT_THREE:
                return getShootThreeAutoCommand();
            case DRIVE_AND_SHOOT_AND_PARK:
                return getDriveAndShootAndParkAutoCommand();
            case DO_NOTHING:
            default:
                return new InstantCommand(); // Retorna um comando vazio que não faz nada.
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
                new LaunchSequenceCommand(shooter, intake), // Reutilizamos o nosso super-comando!
                new FollowPathCommand(drivetrain, parkPath)
        );
    }

    public Command getShootThreeAutoCommand() {
        return new SequentialCommandGroup(
                shooter.spinUpCommand(),
                new WaitUntilCommand(() -> shooter.atTargetVelocity(Constants.Shooter.VELOCITY_TOLERANCE))
                        .withTimeout(3000),
                // Primeiro tiro
                intake.feedCommand(),
                new WaitCommand(500),
                // Segundo tiro
                intake.feedCommand(),
                new WaitCommand(500),
                // Terceiro tiro
                intake.feedCommand(),
                new WaitCommand(250),
                // Parar tudo
                shooter.stopCommand(),
                intake.stopCommand()
        );
    }
}
