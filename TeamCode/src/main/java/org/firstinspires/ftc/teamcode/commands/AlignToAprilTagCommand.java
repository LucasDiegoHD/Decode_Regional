package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class AlignToAprilTagCommand extends CommandBase {

    private final Follower follower;
    private final VisionSubsystem vision;
    private final TelemetryManager telemetry;
    private final PIDFController turnController;

    public AlignToAprilTagCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, TelemetryManager telemetry) {
        this.follower = drivetrain.getFollower();
        this.vision = vision;
        this.telemetry = telemetry;
        this.turnController = new  PIDFController(Constants.Vision.TURN_KP, Constants.Vision.TURN_KI, Constants.Vision.TURN_KD, Constants.Vision.TURN_KF);
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        turnController.reset();
        turnController.setSetPoint(0);
        turnController.setTolerance(0);
    }

    @Override
    public void execute() {
        turnController.setPIDF(
                Constants.Vision.TURN_KP,
                Constants.Vision.TURN_KI,
                Constants.Vision.TURN_KD,
                Constants.Vision.TURN_KF
        );

        if (!vision.hasTarget()) {
            follower.setTeleOpDrive(0, 0, 0, true);
            telemetry.debug("Nenhuma AprilTag detectada");
            telemetry.update();
            return;
        }

        double turnPower = turnController.calculate(vision.getTargetTx().orElse(0.0));
        turnPower = Math.max(-0.4, Math.min(0.4, turnPower));
        telemetry.debug("Align TX: " + vision.getTargetTx().orElse(0.0));
        telemetry.debug("Turn Power: " + turnPower);
        telemetry.update();

        follower.setTeleOpDrive(0, 0, turnPower, true);
    }

    @Override
    public boolean isFinished() {
        return vision.hasTarget() && turnController.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        follower.setTeleOpDrive(0, 0, 0, true);
        telemetry.debug("Alinhamento finalizado.");
        telemetry.update();
    }
}
