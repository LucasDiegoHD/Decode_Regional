package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;

/**
 * Comando para mirar o robô (apenas rotação) em direção a uma coordenada (X, Y)
 * fixa no campo.
 */
public class AimByPoseCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final PIDController angleController;

    public final double targetX;
    public final double targetY;

    private final boolean shooterIsOnBack = true;

    public AimByPoseCommand(DrivetrainSubsystem drivetrain,
                            double targetX,
                            double targetY) {

        this.drivetrain = drivetrain;
        this.targetX = targetX;
        this.targetY = targetY;

        this.angleController = new PIDController(
                ShooterConstants.ANGLE_KP,
                ShooterConstants.ANGLE_KI,
                ShooterConstants.ANGLE_KD
        );

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        angleController.reset();
        angleController.setSetPoint(0);
    }

    @Override
    public void execute() {
        Pose pose = drivetrain.getFollower().getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();

        double heading = MathFunctions.normalizeAngle(pose.getHeading());

        double desired = Math.atan2(targetY - robotY, targetX - robotX);

        if (shooterIsOnBack) {
            desired = MathFunctions.normalizeAngle(desired + Math.PI);
        }

        double error = desired - heading;
        error = Math.atan2(Math.sin(error), Math.cos(error));

        double omega = angleController.calculate(-error);

        System.out.println("ALIGN HEADING (deg): " + Math.toDegrees(heading));
        System.out.println("ALIGN DESIRED (deg): " + Math.toDegrees(desired));
        System.out.println("ALIGN ERROR (deg): " + Math.toDegrees(error));

        drivetrain.getFollower().setTeleOpDrive(0, 0, omega, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}