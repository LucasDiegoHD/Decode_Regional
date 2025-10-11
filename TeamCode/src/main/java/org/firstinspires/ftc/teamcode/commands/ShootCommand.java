package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final Action action;
    private final double targetVelocity;

    public enum Action {
        SPIN_UP,
        STOP
    }

    public ShootCommand(ShooterSubsystem shooter, Action action, double targetVelocity) {
        this.shooter = shooter;
        this.action = action;
        this.targetVelocity = targetVelocity;
        addRequirements(shooter);
    }

    public ShootCommand(ShooterSubsystem shooter, Action action) {
        this(shooter, action, 0);
    }

    @Override
    public void initialize() {
        switch (action) {
            case SPIN_UP:
                shooter.setTargetVelocity(targetVelocity);
                break;
            case STOP:
                shooter.stop();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        if (action == Action.STOP) {
            return true;
        }
        return shooter.atTargetVelocity(ShooterConstants.Shooter.VELOCITY_TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && action == Action.SPIN_UP) {
            shooter.stop();
        }
    }
}