package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final Timer timer = new Timer();
    private enum SHOOT_STATES{
        Conveyor, Acceleration, Triggering, Shooting
    }

    SHOOT_STATES state;
    private int shooterCounter;
    private final TelemetryManager telemetryM;
    public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, int shoots) {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        this.shooterCounter = shoots;
        this.shooter = shooter;
        this.intake = intake;
        timer.resetTimer();
        addRequirements(shooter, intake);
    }
    public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
        this(shooter, intake, 99);
    }

    @Override
    public void initialize() {
        intake.run();
        timer.resetTimer();
        state = SHOOT_STATES.Conveyor;
        telemetryM.addData("Shoot State", state);
    }
    @Override
    public void execute() {
        switch (state) {
            case Conveyor:
                if(timer.getElapsedTime()> ShooterConstants.INTAKE_TIMER_TO_SHOOT){
                    state = SHOOT_STATES.Acceleration;
                    timer.resetTimer();
                    intake.stop();
                }
                break;
            case Acceleration:
                if (shooter.getShooterAtTarget()) {
                    state = SHOOT_STATES.Triggering;
                    timer.resetTimer();
                    intake.runTrigger();
                }
                break;
            case Triggering:
                if (!shooter.getShooterAtTarget()) {
                    timer.resetTimer();
                    state = SHOOT_STATES.Shooting;
                }
                break;
            case Shooting:
                if (timer.getElapsedTime() > ShooterConstants.TRIGGER_TIMER_TO_SHOOT) {
                    state = SHOOT_STATES.Conveyor;
                    timer.resetTimer();
                    intake.stopTrigger();
                    intake.run();
                    if (shooterCounter > 0) {
                        shooterCounter--;
                    }
                }

        }
        telemetryM.addData("Shoot State", state);

    }

    @Override
    public boolean isFinished() {
        return shooterCounter == 0;
    }
    @Override
    public void end(boolean interrupted){
        intake.stopTrigger();
        intake.stop();
        telemetryM.addData("Shoot State", "Finish");

    }
}
