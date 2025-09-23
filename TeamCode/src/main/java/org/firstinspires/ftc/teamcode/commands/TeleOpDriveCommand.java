package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class TeleOpDriveCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final GamepadEx driver;
    private Follower follower;


    public TeleOpDriveCommand(DrivetrainSubsystem drivetrain, GamepadEx driver, Follower follower) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        this.follower = follower;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Obtenha a direção do robô a partir do subsistema Drivetrain
        double botHeading = drivetrain.getFollower().getHeading();

        double x = driver.getLeftX();
        double y = -driver.getLeftY(); // O Y no gamepad é invertido
        double rx = driver.getRightX();

        // Rotação dos vetores de movimento
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        follower = Constants.Drivetrain.createFollower(hardwareMap);


    }
}