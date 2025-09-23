package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class AlignToAprilTagCommand extends CommandBase {

    private final Follower follower;
    private final VisionSubsystem vision;
    private final TelemetryManager telemetry;
    private final PIDController turnController;

    public AlignToAprilTagCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        this.follower = drivetrain.getFollower();
        this.vision = vision;
        this.telemetry = telemetry;
        this.turnController = new PIDController(Constants.Vision.TURN_KP, Constants.Vision.TURN_KI, Constants.Vision.TURN_KD);
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {turnController.reset();turnController.setSetPoint(0);turnController.setTolerance(0.5);}

    @Override
    public void execute() {
        turnController.setPID(
                Constants.Vision.TURN_KP,
                Constants.Vision.TURN_KI,
                Constants.Vision.TURN_KD
        );

        if (!vision.hasTarget()) {
            follower.setTeleOpDrive(0, 0, 0, true);
            telemetry.addLine("Nenhuma AprilTag detectada");
            return;
        }

        double tx = vision.getTargetTx().orElse(0.0);

        double turnPower = turnController.calculate(tx);
        // limita a potência pra não travar motor
        turnPower = Math.max(-0.4, Math.min(0.4, turnPower));

        telemetry.addData("Align TX", tx);
        telemetry.addData("Turn Power", turnPower);

        follower.setTeleOpDrive(0, 0, turnPower, true);
    }

    @Override
    public boolean isFinished() {
        return turnController.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        follower.setTeleOpDrive(0, 0, 0, true);
    }
}
