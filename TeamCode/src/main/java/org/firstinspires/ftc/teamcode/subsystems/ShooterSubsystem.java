package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private final Motor shooterMotorRight;
    private final Motor shooterMotorLeft;
    private final PIDController pidController;
    private double targetVelocity;

    private final TelemetryManager telemetry;

    public ShooterSubsystem(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        shooterMotorRight = new Motor(hardwareMap, "shooterRight", Motor.GoBILDA.RPM_1150);
        shooterMotorLeft = new Motor(hardwareMap, "shooterLeft", Motor.GoBILDA.RPM_1150);
        pidController = new PIDController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD);
        targetVelocity = 0;

        shooterMotorRight.setRunMode(Motor.RunMode.VelocityControl);
    }

    @Override
    public void periodic() {
        if (targetVelocity != 0) {
            double power = pidController.calculate(getCurrentVelocity(), targetVelocity);
            shooterMotorRight.set(power);
            shooterMotorLeft.set(-power);
        } else {
            shooterMotorRight.stopMotor();
            shooterMotorLeft.stopMotor();
        }

        telemetry.addData("Shooter Velocity", getCurrentVelocity());
        telemetry.addData("Shooter Target Velocity", targetVelocity);
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public double getCurrentVelocity() {

        return shooterMotorRight.getCorrectedVelocity();
    }

    public boolean atTargetVelocity(double tolerance) {
        return Math.abs(getCurrentVelocity() - targetVelocity) < tolerance;
    }

    public void stop() {
        targetVelocity = 0;
    }

    // --- FÁBRICAS DE COMANDOS ---

    public Command spinUpCommand() {
        // Agora o comando de atirar recebe apenas o subsistema de atirador
        // e a velocidade alvo.
        return new ShootCommand(this, ShootCommand.Action.SPIN_UP, Constants.Shooter.TARGET_VELOCITY);
    }

    public Command stopCommand() {
        // O comando de parar também precisa apenas do subsistema de atirador.
        return new ShootCommand(this, ShootCommand.Action.STOP, Constants.Shooter.TARGET_VELOCITY);
    }
}