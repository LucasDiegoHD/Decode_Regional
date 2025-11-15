package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * A command to automatically align the robot to an AprilTag target.
 * It uses a PID controller to turn the robot towards the target while allowing the driver to maintain
 * forward/backward and strafe control. It also calculates the appropriate shooter hood position.
 */
public class AlignToAprilTagCommand extends CommandBase {

    private final Follower follower;
    private final VisionSubsystem vision;
    private final TelemetryManager telemetry;
    private final PIDFController turnController;
    private int IsAprilTagNotSeemCounter = 0;
    private static final int ApriltagNotSeemMaximumCounter = 20;

    private final GamepadEx driverGamepad;


    /**
     * Creates a new AlignToAprilTagCommand.
     *
     * @param drivetrain    The DrivetrainSubsystem to control.
     * @param vision        The VisionSubsystem to get target data from.
     * @param telemetry     The TelemetryManager for logging.
     * @param driverGamepad The driver's gamepad for movement input.
     */
    public AlignToAprilTagCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, TelemetryManager telemetry, GamepadEx driverGamepad) {
        this.follower = drivetrain.getFollower();
        this.vision = vision;
        this.telemetry = telemetry;
        this.driverGamepad = driverGamepad;
        this.turnController = new  PIDFController(VisionConstants.TURN_KP, VisionConstants.TURN_KI, VisionConstants.TURN_KD, VisionConstants.TURN_KF);
        addRequirements(drivetrain, vision);
    }

    /**
     * Called when the command is initially scheduled. Resets the PID controller.
     */
    @Override
    public void initialize() {
        turnController.reset();
        turnController.setTolerance(0.2);
    }

    /**
     * Called repeatedly while the command is scheduled. Calculates and applies turn power
     * to align with the target, while also processing driver joystick inputs.
     */
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
            telemetry.debug("No AprilTag detected");
            telemetry.update();
            IsAprilTagNotSeemCounter++;
        } else {
            IsAprilTagNotSeemCounter = 0;
        }

        // Allow driver to control forward/backward and strafe while aligning
        double heading = follower.getHeading();
        double y = 0;
        double x = 0;
        if (driverGamepad != null) {
            y = driverGamepad.getLeftY(); // forward/backward
            x = -driverGamepad.getLeftX(); // strafe
        }
        double xField = x * Math.cos(heading) - y * Math.sin(heading);
        double yField = x * Math.sin(heading) + y * Math.cos(heading);

        // Calculate turn power using PID on the vision target's horizontal offset (tx)
        double turnPower = turnController.calculate(vision.getTargetTx().orElse(0.0));
        turnPower = Math.max(-0.4, Math.min(0.4, turnPower)); // Clamp turn power

        telemetry.debug("Align TX: " + vision.getTargetTx().orElse(0.0));
        telemetry.debug("Turn Power: " + turnPower);


        follower.setTeleOpDrive(yField, xField, turnPower, true);


    }

    /**
     * Returns true when the command should end.
     *
     * @return True if the robot is aligned to the target, or if the target is lost for too long.
     */
    @Override
    public boolean isFinished() {
        return (vision.hasTarget() && turnController.atSetPoint()) || IsAprilTagNotSeemCounter > ApriltagNotSeemMaximumCounter;
    }

    /**
     * Called when the command ends or is interrupted. Stops the drivetrain.
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        follower.setTeleOpDrive(0, 0, 0, true);
        telemetry.debug("Alignment finished.");
        telemetry.update();
    }
}
