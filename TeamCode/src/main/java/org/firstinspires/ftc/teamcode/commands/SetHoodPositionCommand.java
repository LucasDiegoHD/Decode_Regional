package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Comando para definir a posição do servo do capuz (hood) do shooter.
 */
public class SetHoodPositionCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final double targetPosition;

    /**
     * @param shooter O subsistema do shooter.
     * @param targetPosition A posição alvo do servo (valor entre 0.0 e 1.0).
     */
    public SetHoodPositionCommand(ShooterSubsystem shooter, double targetPosition) {
        this.shooter = shooter;
        this.targetPosition = targetPosition;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Define a posição do servo imediatamente.
        //shooter.setHoodPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        // Este comando termina assim que a posição é definida.
        return true;
    }
}
