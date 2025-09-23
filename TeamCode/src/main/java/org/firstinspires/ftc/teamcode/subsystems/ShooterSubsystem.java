package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx shooterMotor;
    private final PIDFController pidfController;
    private final TelemetryManager telemetry;
    private double targetVelocity = 0.0;

    public ShooterSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        shooterMotor = hardwareMap.get(DcMotorEx.class, Constants.Shooter.SHOOTER_MOTOR_NAME);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        pidfController = new PIDFController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD, Constants.Shooter.kF);
    }

    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
    }

    public void stop() {
        setTargetVelocity(0);
    }

    public boolean atTargetVelocity(double tolerance) {
        return Math.abs(getCurrentVelocity() - targetVelocity) < tolerance;
    }

    public double getCurrentVelocity() {
        return shooterMotor.getVelocity();
    }

    @Override
    public void periodic() {
        pidfController.setPIDF(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD, Constants.Shooter.kF);

        if (targetVelocity!= 0) {
            double power = pidfController.calculate(getCurrentVelocity(), targetVelocity);
            shooterMotor.setPower(power);
        } else {
            shooterMotor.setPower(0);
        }

    }

    // --- FÁBRICAS DE COMANDOS ---
    /**
     * Retorna um comando que LIGA o shooter para a velocidade alvo.
     * Usa o ShootCommand que já tínhamos, mas agora é mais claro o seu propósito.
     */
    public Command spinUpCommand() {
        return new ShootCommand(this, ShootCommand.Action.SPIN_UP, Constants.Shooter.TARGET_VELOCITY);
    }

    /**
     * Retorna um comando que PARA o shooter.
     */
    public Command stopCommand() {
        return new ShootCommand(this, ShootCommand.Action.STOP);
    }
}
