package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.Command;
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
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.DriveInTriangleCommand;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class RobotContainer {

    private final DrivetrainSubsystem drivetrain;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final TelemetryManager telemetry;
    private Follower follower;

    Pose startPose = new Pose(0, 0, Math.toRadians(-60));
    Pose shootingPose = new Pose(12, 60, Math.toRadians(90));
    Pose parkPose = new Pose(120, 10, Math.toRadians(0));

    public RobotContainer(HardwareMap hardwareMap, TelemetryManager telemetry, GamepadEx driver, GamepadEx operator) {
        this.telemetry = telemetry;
        this.follower = follower;
        vision = new VisionSubsystem(hardwareMap, telemetry);
        drivetrain = new DrivetrainSubsystem(hardwareMap, telemetry, vision);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);

        if (driver != null) {
            drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, driver, follower));

            new GamepadButton(driver, GamepadKeys.Button.Y)
                    .whileHeld(new AlignToAprilTagCommand(drivetrain, vision, telemetry));


            new GamepadButton(driver, GamepadKeys.Button.X)
                    .whenPressed(new SequentialCommandGroup(
                            // Obtém a pose atual do robô.
                            new InstantCommand(() -> {
                                Pose currentPose = drivetrain.getFollower().getPose();

                                PathChain parkPath = drivetrain.getFollower().pathBuilder()
                                        .addPath(new BezierLine(currentPose, parkPose))
                                        .setLinearHeadingInterpolation(currentPose.getHeading(), parkPose.getHeading())
                                        .build();

                                // Agenda o comando para seguir o caminho recém-criado.
                                // O TelemetryManager é passado para o comando.
                                new FollowPathCommand(drivetrain, parkPath, telemetry).schedule();
                            }),
                            // Adiciona uma pequena espera para a execução do comando de FollowPath
                            new WaitCommand(100),
                            // Retorna o comando de controle do driver
                            new InstantCommand(() -> drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, driver, follower)))
                    ));

            // Adicionado novo binding para o botão A do gamepad do driver
            // Agora o botão A dispara o novo comando que contém a lógica do triângulo
            new GamepadButton(driver, GamepadKeys.Button.A)
                    .whenPressed(new SequentialCommandGroup(
                            new DriveInTriangleCommand(drivetrain, telemetry),
                            new InstantCommand(() -> drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, driver, follower)))
                    ));
        }
        if (operator != null) {
            configureTeleOpBindings(operator, telemetry);
        }
    }

    private void configureTeleOpBindings(GamepadEx operator, TelemetryManager telemetry) {
        new GamepadButton(operator, GamepadKeys.Button.B)
                .whenPressed(shooter.spinUpCommand());

        new GamepadButton(operator, GamepadKeys.Button.X)
                .whenPressed(shooter.stopCommand());


    }


    /**
     * Esta rotina autônoma completa realiza o movimento para a posição de tiro,
     * aguarda o shooter atingir a velocidade, realiza um tiro e, por fim, se move para a área de parking.
     * @return O grupo de comandos sequenciais para a rotina.
     */
    public Command getAutoAvancadoCommand() {
        // Define a pose inicial do robô.
        drivetrain.getFollower().setStartingPose(startPose);

        // Constrói o caminho que leva o robô da posição inicial até a posição de tiro.
        PathChain driveAndShootPath = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                // Adiciona um callback para iniciar o shooter quando o robô estiver a 70% do caminho.
                // Isso economiza tempo e garante que o shooter esteja pronto para o tiro.
                .addParametricCallback(0.7, () -> shooter.spinUpCommand().schedule())
                .build();

        // Constrói o caminho para a posição de parking.
        PathChain parkPath = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(shootingPose, parkPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), parkPose.getHeading())
                .build();

        // Retorna um grupo de comandos sequenciais para executar cada etapa da rotina.
        return new SequentialCommandGroup(
                // 1. Segue o caminho para a posição de tiro. O shooter já deve ter sido acionado pelo callback.
                new FollowPathCommand(drivetrain, driveAndShootPath, telemetry),

                // 2. Aguarda até que o shooter atinja a velocidade alvo (com um timeout de segurança de 2s).
                new WaitUntilCommand(() -> shooter.atTargetVelocity(Constants.Shooter.VELOCITY_TOLERANCE))
                        .withTimeout(2000),

                // 3. Empurra a peça para ser atirada. Substitua por um comando real de seu subsistema de Intake.
                // Exemplo: `intake.feedCommand()`
                new InstantCommand(() -> {
                    System.out.println("Peça sendo atirada!");
                }, intake),

                // 4. Aguarda um curto período para a peça sair do shooter.
                new WaitCommand(250),

                // 5. Para o shooter.
                shooter.stopCommand(),

                // 6. Segue o caminho para a posição de parking.
                new FollowPathCommand(drivetrain, parkPath, telemetry)
        );
    }

    /**
     * Esta rotina autônoma é um exemplo de como atirar 3 peças em sequência
     * sem a necessidade de movimentação de pathing.
     * @return O grupo de comandos sequenciais para atirar as peças.
     */
    public Command getShootThreeAutoCommand() {
        return new SequentialCommandGroup(
                // 1. Gira o shooter para a velocidade alvo.
                shooter.spinUpCommand(),

                // 2. Espera ATÉ que o shooter atinja a velocidade (com um timeout de segurança de 3s).
                new WaitUntilCommand(() -> shooter.atTargetVelocity(Constants.Shooter.VELOCITY_TOLERANCE))
                        .withTimeout(3000),

                // 3. Realiza o primeiro tiro.
                new InstantCommand(() -> { /* Substituir por intake.feedCommand() */ }, intake, shooter),

                // 4. Espera um pouco para o shooter recuperar a velocidade.
                new WaitCommand(500),

                // --- SEGUNDO TIRO ---
                // 5. Empurra o segundo elemento.
                new InstantCommand(() -> { /* Substituir por intake.feedCommand() */ }, intake, shooter),
                new WaitCommand(500),

                // --- TERCEIRO TIRO ---
                // 6. Empurra o terceiro elemento.
                new InstantCommand(() -> { /* Substituir por intake.feedCommand() */ }, intake, shooter),
                new WaitCommand(250), // Espera final um pouco menor

                // 7. Para o shooter.
                shooter.stopCommand()
        );
    }
}
