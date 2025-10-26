package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class SpinShooterCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final Action action;
    public enum Action {
        LONG_SHOOT,
        SHORT_SHOOT,
        STOP
    }

    public SpinShooterCommand(ShooterSubsystem shooter, Action action) {
        this.shooter = shooter;
        this.action = action;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        switch (action) {
            case LONG_SHOOT:
                // CORREÇÃO: Passa o targetVelocity (RPM) para o subsistema
                shooter.setTargetVelocity(ShooterConstants.TARGET_VELOCITY_LONG);
                break;
            case SHORT_SHOOT:
                shooter.setTargetVelocity(ShooterConstants.TARGET_VELOCITY_SHORT);
                break;
            case STOP:
                shooter.stop();
                break;
        }
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
