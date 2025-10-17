package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class SpinShooterCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final Action action;
    private final double targetVelocity; // Velocidade (RPM) que será usada

    public enum Action {
        SPIN_UP,
        STOP
    }

    public SpinShooterCommand(ShooterSubsystem shooter, Action action, double targetVelocity) {
        this.shooter = shooter;
        this.action = action;
        this.targetVelocity = targetVelocity;
        addRequirements(shooter);
    }

    public SpinShooterCommand(ShooterSubsystem shooter, Action action) {
        // Construtor para comandos que não precisam de uma velocidade (ex: STOP)
        this(shooter, action, 0);
    }

    @Override
    public void initialize() {
        switch (action) {
            case SPIN_UP:
                // CORREÇÃO: Passa o targetVelocity (RPM) para o subsistema
                shooter.setTargetVelocity(targetVelocity);
                break;
            case STOP:
                shooter.stop();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // O comando de "SPIN_UP" deve rodar até ser interrompido (ex: pela próxima ação do jogador)
        // Se você quiser esperar até o shooter atingir a velocidade, a lógica seria adicionada aqui.
        // Por enquanto, ele roda indefinidamente até que um novo comando tome o shooter.
        return action == Action.STOP;
    }
}
