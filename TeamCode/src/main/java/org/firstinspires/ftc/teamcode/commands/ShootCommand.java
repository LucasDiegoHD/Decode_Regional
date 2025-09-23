package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final Action action;
    private final double targetVelocity;

    public ShootCommand(ShooterSubsystem shooter, Action action, double targetVelocity, ShooterSubsystem shooter1, DrivetrainSubsystem drivetrainSubsystem, Action action1, double targetVelocity1) {
        this.shooter = shooter1;
        this.action = action1;
        this.targetVelocity = targetVelocity1;
    }

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
        // Retorna true se o motor está na velocidade alvo.
        return shooter.atTargetVelocity(Constants.Shooter.VELOCITY_TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && action == Action.SPIN_UP) {
            shooter.stop();
        }
    }
}
