package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final Timer timer = new Timer();

    public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.intake = intake;
        timer.resetTimer();
        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        intake.run();
        timer.resetTimer();
    }
    @Override
    public void execute() {
        if(timer.getElapsedTime()> ShooterConstants.INTAKE_TIMER_TO_SHOOT) {
            intake.stop();
            if (shooter.getShooterAtTarget()) {
                intake.runTrigger();
            } else {
                intake.stopTrigger();
                timer.resetTimer();
            }
        }
        else{
            intake.run();
        }
    }
    @Override
    public void end(boolean interrupted){
        intake.stopTrigger();
        intake.stop();
    }
}
