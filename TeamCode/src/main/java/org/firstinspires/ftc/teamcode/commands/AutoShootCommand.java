package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * This command automates the entire shooting sequence with a single button press.
 * It aligns to the target, adjusts the shooter, waits for it to be ready, and then launches the note.
 */
public class AutoShootCommand extends SequentialCommandGroup {

    public AutoShootCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, TelemetryManager telemetry, GamepadEx operator) {
        addCommands(
                // Step 1: Aim the robot and spin up the shooter to the correct speed.
                new AlignToAprilTagCommand(drivetrain, vision, shooter, telemetry, operator),

                // Step 2: Wait until the shooter is at the target speed (with a 2-second safety timeout).
                new WaitUntilCommand(shooter::getShooterAtTarget).withTimeout(2000),

                // Step 3: Feed the game piece through the trigger.
                new InstantCommand(intake::runTrigger, intake),
                new WaitCommand(400), // Wait for the piece to pass through
                new InstantCommand(intake::stopTrigger, intake)

                // Note: The shooter is left running after the sequence.
                // This allows for rapid subsequent shots. A separate button is used to stop it.
        );
    }
}
