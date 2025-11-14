package org.firstinspires.ftc.teamcode.autos.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.SpinShooterCommand;
import org.firstinspires.ftc.teamcode.commands.UpdatePoseLimelightCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.List;

public class AutonomousLeave extends SequentialCommandGroup {


    public AutonomousLeave(@NonNull DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, VisionSubsystem vision, List<Pose> poses) {

        addCommands(
                new AutonomousCommands(drivetrain, shooter, intake, indexer, vision, poses).withTimeout(28000),
                new LeaveCommand(drivetrain)

        );
        addRequirements(drivetrain, shooter, intake);
    }

}

