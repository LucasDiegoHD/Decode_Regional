package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.SetHoodPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class RobotContainer {

    private final DrivetrainSubsystem drivetrain;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final IMU imu;


    Pose startPose = new Pose(0, 0, Math.toRadians(-60));
    Pose shootingPose = new Pose(12, 60, Math.toRadians(90));
    Pose parkPose = new Pose(10, 120, Math.toRadians(0));

    public RobotContainer(HardwareMap hardwareMap, TelemetryManager telemetry, GamepadEx driver, GamepadEx operator) {
        drivetrain = new DrivetrainSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        vision = new VisionSubsystem(hardwareMap, telemetry);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(myIMUparameters);
        drivetrain.getFollower().setPose(vision.getRobotPose(0).orElse(new Pose()));
        if (driver != null) {
            drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, driver, imu));

            new GamepadButton(driver, GamepadKeys.Button.Y)
                    .whileHeld(new AlignToAprilTagCommand(drivetrain, vision, shooter, telemetry));


            /*new GamepadButton(driver, GamepadKeys.Button.X)
                    .whenPressed(new SequentialCommandGroup(
                            // Obtém a pose atual do robô.
                            new InstantCommand(() -> {
                                Pose currentPose = drivetrain.getFollower().getPose();

                                PathChain parkPath = drivetrain.getFollower().pathBuilder()
                                        .addPath(new BezierLine(currentPose, parkPose))
                                        .setLinearHeadingInterpolation(currentPose.getHeading(), parkPose.getHeading())
                                        .build();

                                // Agenda o comando para seguir o caminho recém-criado.
                                // Este comando será executado como parte do grupo sequencial.
                                new FollowPathCommand(drivetrain, parkPath).schedule();
                            }),
                            // Adiciona uma pequena espera para a execução do comando de FollowPath
                            new WaitCommand(100),
                            // Retorna o comando de controle do driver
                            new InstantCommand(() -> drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, driver)))
                    ));*/
        }
        if (operator != null) {
            configureTeleOpBindings(operator, telemetry);

        }
    }

    private void configureTeleOpBindings(GamepadEx operator, TelemetryManager telemetry) {



        new GamepadButton(operator, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(shooter::decreaseHood, shooter));
        new GamepadButton(operator, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(shooter::increaseHood, shooter));
        new GamepadButton(operator, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(intake::runTrigger, intake))
                .whenReleased(new InstantCommand(intake::stopTrigger, intake));

        new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(shooter::stop, shooter));

        new GamepadButton(operator, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(intake::run, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));
        new GamepadButton(operator, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(intake::reverse, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));
        new GamepadButton(operator, GamepadKeys.Button.B)
                .whenPressed(new ShootCommand(shooter, ShootCommand.Action.SPIN_UP, ShooterConstants.TARGET_VELOCITY));
        new GamepadButton(operator,GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(()->shooter.setTargetVelocity(6000),shooter));
    }

}
