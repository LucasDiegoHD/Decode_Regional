package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class AlignToAprilTagCommand extends CommandBase {

    private final Follower follower;
    private final VisionSubsystem vision;
    private final ShooterSubsystem shooter;
    private final TelemetryManager telemetry;
    private final PIDFController turnController;
    private int IsAprilTagNotSeemCounter = 0;
    private static final int ApriltagNotSeemMaximumCounter = 20;

    public AlignToAprilTagCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, TelemetryManager telemetry) {
        this.follower = drivetrain.getFollower();
        this.vision = vision;
        this.shooter = shooter;
        this.telemetry = telemetry;
        this.turnController = new  PIDFController(VisionConstants.TURN_KP, VisionConstants.TURN_KI, VisionConstants.TURN_KD, VisionConstants.TURN_KF);
        addRequirements(drivetrain, vision, shooter);
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
                VisionConstants.TURN_KP,
                VisionConstants.TURN_KI,
                VisionConstants.TURN_KD,
                VisionConstants.TURN_KF
        );

        if (!vision.hasTarget()) {
            follower.setTeleOpDrive(0, 0, 0, true);
            telemetry.debug("Nenhuma AprilTag detectada");
            telemetry.update();
            IsAprilTagNotSeemCounter++;
        }
        else{
            IsAprilTagNotSeemCounter = 0;
        }

        double turnPower = turnController.calculate(vision.getTargetTx().orElse(0.0));
        turnPower = Math.max(-0.4, Math.min(0.4, turnPower));
        telemetry.debug("Align TX: " + vision.getTargetTx().orElse(0.0));
        telemetry.debug("Turn Power: " + turnPower);


        follower.setTeleOpDrive(0, 0, turnPower, true);
        double hoodPosition = ShooterConstants.MINIMUM_HOOD +(ShooterConstants.MAXIMUM_HOOD-ShooterConstants.MINIMUM_HOOD) *
                ((VisionConstants.MAXIMUM_TA-vision.getTargetTa().orElse(0.0))/(VisionConstants.MAXIMUM_TA-VisionConstants.MINIMUM_TA));

        //shooter.setHoodPosition(hoodPosition);
        telemetry.addData("Hood Setting",hoodPosition);
    }

    @Override
    public boolean isFinished() {

        return (vision.hasTarget() && turnController.atSetPoint()) || IsAprilTagNotSeemCounter>ApriltagNotSeemMaximumCounter;
    }

    @Override
    public void end(boolean interrupted) {
        follower.setTeleOpDrive(0, 0, 0, true);
        telemetry.debug("Alinhamento finalizado.");
        telemetry.update();
    }
}
