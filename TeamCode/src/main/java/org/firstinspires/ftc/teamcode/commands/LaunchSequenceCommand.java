package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Um "Super-Comando" que encapsula toda a sequência de lançamento.
 * 1. Acelera o shooter.
 * 2. Espera até atingir a velocidade alvo.
 * 3. Alimenta a peça de jogo.
 * 4. Espera um pouco.
 * 5. Para tudo.
 * Isto torna o RobotContainer extremamente limpo e a lógica reutilizável.
 */
public class LaunchSequenceCommand extends SequentialCommandGroup {

    public LaunchSequenceCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
        addCommands(
                // 1. Inicia o shooter (não bloqueia, corre em segundo plano)
                shooter.spinUpCommand(),

                // 2. Espera ATÉ que o shooter atinja a velocidade (com um timeout de segurança)
                new WaitUntilCommand(() -> shooter.atTargetVelocity(Constants.Shooter.VELOCITY_TOLERANCE))
                        .withTimeout(2000),

                // 3. Alimenta a peça de jogo para o shooter
                intake.feedCommand(),

                // 4. Espera um curto período para a peça sair completamente
                new WaitCommand(250),

                // 5. Para ambos os subsistemas
                shooter.stopCommand(),
                intake.stopCommand()
        );

        // Adiciona os requisitos para garantir que nenhum outro comando interfira
        addRequirements(shooter, intake);
    }
}
