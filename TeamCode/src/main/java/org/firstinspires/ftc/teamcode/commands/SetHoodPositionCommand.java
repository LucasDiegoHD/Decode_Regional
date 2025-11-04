package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * A command to set the position of the shooter's hood servo.
 */
public class SetHoodPositionCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final double targetPosition;

    /**
     * Creates a new SetHoodPositionCommand.
     *
     * @param shooter The ShooterSubsystem to use.
     * @param targetPosition The target position for the servo (a value between 0.0 and 1.0).
     */
    public SetHoodPositionCommand(ShooterSubsystem shooter, double targetPosition) {
        this.shooter = shooter;
        this.targetPosition = targetPosition;
        addRequirements(shooter);
    }

    /**
     * Called when the command is initially scheduled. Sets the hood servo's position.
     * Note: The underlying method call in the subsystem is currently commented out.
     */
    @Override
    public void initialize() {
        // shooter.setHoodPosition(targetPosition);
    }

    /**
     * Returns true when the command should end.
     *
     * @return True immediately, as this is an instant command.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}
