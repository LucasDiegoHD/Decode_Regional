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
    private final IMU imu;


    public TeleOpDriveCommand(DrivetrainSubsystem drivetrain, GamepadEx driverGamepad, IMU imu) {
        this.drivetrain = drivetrain;
        this.driverGamepad = driverGamepad;
        this.imu = imu;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.getFollower().startTeleopDrive();
    }

    @Override
    public void execute() {
        if(driverGamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
            imu.resetYaw();
        }
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double y = driverGamepad.getLeftY(); // frente/trás
        double x =  -driverGamepad.getLeftX(); // lateral

        double xField = x * Math.cos(heading) - y * Math.sin(heading);
        double yField = x * Math.sin(heading) + y * Math.cos(heading);

        drivetrain.getFollower().setTeleOpDrive(
                yField,
                xField,
                -driverGamepad.getRightX(),
                true
        );
    }

}