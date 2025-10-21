package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

/**
 * A teleoperated drive command that allows the robot to move using field-centric control
 * while automatically aiming toward a specified target point (X, Y).
 * <p>
 * The driver can override the automatic rotation control using the right stick.
 * This command integrates an IMU-based heading correction for field-oriented movement
 * and a PID controller for rotational aiming.
 */
public class TeleOpDriveAimingCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final GamepadEx driverGamepad;
    private final IMU imu;
    private final PIDController angleController;

    public final double targetX;
    public final double targetY;

    private static final double ANGLE_KP = 0.3;
    private static final double ANGLE_KI = 0.0;
    private static final double ANGLE_KD = 0.0;

    /**
     * Constructs a new {@code TeleOpDriveAimingCommand}.
     *
     * @param drivetrain    The drivetrain subsystem used to control the robot's movement.
     * @param driverGamepad The driver's gamepad for manual input.
     * @param imu           The IMU sensor providing the robot's heading.
     * @param targetX       The X-coordinate of the target point to aim toward.
     * @param targetY       The Y-coordinate of the target point to aim toward.
     */
    public TeleOpDriveAimingCommand(DrivetrainSubsystem drivetrain,
                                    GamepadEx driverGamepad,
                                    IMU imu,
                                    double targetX,
                                    double targetY) {
        this.drivetrain = drivetrain;
        this.driverGamepad = driverGamepad;
        this.imu = imu;
        this.targetX = targetX;
        this.targetY = targetY;
        this.angleController = new PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD);

        addRequirements(drivetrain);
    }

    /** Initializes the command by resetting controllers and preparing teleop drive mode. */
    @Override
    public void initialize() {
        drivetrain.getFollower().startTeleopDrive();
        angleController.reset();
    }

    /**
     * Executes one iteration of the teleoperated drive logic.
     * Handles driver input, field-centric movement, and rotation toward the target point.
     */
    @Override
    public void execute() {
        if (driverGamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
            imu.resetYaw();
        }

        double forwardInput = -driverGamepad.getLeftY();
        double strafeInput = driverGamepad.getLeftX();
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Convert driver inputs to field-centric coordinates
        double xField = strafeInput * Math.cos(heading) - forwardInput * Math.sin(heading);
        double yField = strafeInput * Math.sin(heading) + forwardInput * Math.cos(heading);

        Pose pose = drivetrain.getFollower().getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = heading;

        // Calculate desired heading toward the target
        double desiredTheta = Math.atan2(targetY - robotY, targetX - robotX);
        double error = desiredTheta - robotHeading;
        error = Math.atan2(Math.sin(error), Math.cos(error)); // Normalize to [-π, π]

        double omega = angleController.calculate(0, error);

        // Allow manual override of rotation
        double manualTurn = -driverGamepad.getRightX();
        if (Math.abs(manualTurn) > 0.05) {
            omega = manualTurn;
        }

        drivetrain.getFollower().setTeleOpDrive(yField, xField, omega, true);
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
