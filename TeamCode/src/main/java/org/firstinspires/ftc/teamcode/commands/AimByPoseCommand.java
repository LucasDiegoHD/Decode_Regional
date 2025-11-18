package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;

/**
 * A teleoperated drive command that allows the robot to move using field-centric control
 * while automatically aiming toward a specified target point (X, Y).
 * <p>
 * The driver can override the automatic rotation control using the right stick.
 * This command integrates an IMU-based heading correction for field-oriented movement
 * and a PID controller for rotational aiming.
 */
public class AimByPoseCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final PIDController angleController;

    public final double targetX;
    public final double targetY;



    /**
     * Constructs a new {@code TeleOpDriveAimingCommand}.
     *
     * @param drivetrain The drivetrain subsystem used to control the robot's movement.
     * @param targetX    The X-coordinate of the target point to aim toward.
     * @param targetY    The Y-coordinate of the target point to aim toward.
     */
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
        angleController.setPID(
                ShooterConstants.ANGLE_KP,
                ShooterConstants.ANGLE_KI,
                ShooterConstants.ANGLE_KD
        );

        angleController.reset();
    }

    @Override
    public void execute() {


        Pose pose = drivetrain.getFollower().getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();

        double headingPedro = drivetrain.getFollower().getHeading();

        double heading = -(headingPedro + Math.PI);
        heading = Math.atan2(Math.sin(heading), Math.cos(heading));

        double desired = Math.atan2(targetY - robotY, targetX - robotX);
        desired = Math.atan2(Math.sin(desired), Math.cos(desired));

        double error = desired - heading;
        error = Math.atan2(Math.sin(error), Math.cos(error));

        System.out.println("ALIGN HEADING: " + heading);
        System.out.println("ALIGN DESIRED: " + desired);
        System.out.println("ALIGN ERROR: " + error);

        double omega = angleController.calculate(error);

        drivetrain.getFollower().setTeleOpDrive(0, 0, omega, true);


    }


    /**
     * Indicates whether the command has completed execution.
     *
     * @return Always {@code false} — this command runs continuously during TeleOp.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
