package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class TeleOpDriveCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final GamepadEx driverGamepad;


    public TeleOpDriveCommand(DrivetrainSubsystem drivetrain, GamepadEx driverGamepad) {
        this.drivetrain = drivetrain;
        this.driverGamepad = driverGamepad;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.getFollower().startTeleopDrive();
    }

    @Override
    public void execute() {
        Pose p = drivetrain.getFollower().getPose();
        double heading = p.getHeading();
        double y = driverGamepad.getLeftX(); // frente/trás
        double x =  driverGamepad.getLeftY(); // lateral

        double xField = x * Math.cos(heading) - y * Math.sin(heading);
        double yField = x * Math.sin(heading) + y * Math.cos(heading);

        drivetrain.getFollower().setTeleOpDrive(
                yField,
                xField,
                -driverGamepad.getRightX(),
                true
        );
        if(driverGamepad.getButton(GamepadKeys.Button.START)){

            drivetrain.getFollower().setPose(p.setHeading(Math.PI));
        }

    }

}