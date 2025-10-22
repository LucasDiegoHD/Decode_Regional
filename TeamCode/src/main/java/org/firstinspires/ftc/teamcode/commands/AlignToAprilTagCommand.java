package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class AlignToAprilTagCommand extends CommandBase {

    private final Follower follower;
    private final VisionSubsystem vision;
    private final ShooterSubsystem shooter;
    private final TelemetryManager telemetry;
    private final PIDFController turnController;
    private int IsAprilTagNotSeemCounter = 0;
    private static final int ApriltagNotSeemMaximumCounter = 20;
    private final IMU imu;
    private final GamepadEx driverGamepad;


    public AlignToAprilTagCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, TelemetryManager telemetry, IMU imu, GamepadEx driverGamepad) {
        this.follower = drivetrain.getFollower();
        this.vision = vision;
        this.shooter = shooter;
        this.telemetry = telemetry;
        this.imu = imu;
        this.driverGamepad = driverGamepad;
        this.turnController = new  PIDFController(VisionConstants.TURN_KP, VisionConstants.TURN_KI, VisionConstants.TURN_KD, VisionConstants.TURN_KF);
        addRequirements(drivetrain, vision, shooter);
    }

    @Override
    public void initialize() {
        turnController.reset();
        turnController.setSetPoint(0);
        turnController.setTolerance(0);
    }

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
            telemetry.debug("Nenhuma AprilTag detectada");
            telemetry.update();
            IsAprilTagNotSeemCounter++;
        }
        else{
            IsAprilTagNotSeemCounter = 0;
        }
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double y = driverGamepad.getLeftY(); // frente/trás
        double x =  -driverGamepad.getLeftX(); // lateral

        double xField = x * Math.cos(heading) - y * Math.sin(heading);
        double yField = x * Math.sin(heading) + y * Math.cos(heading);
        double turnPower = turnController.calculate(vision.getTargetTx().orElse(0.0));
        turnPower = Math.max(-0.3, Math.min(0.3, turnPower));
        telemetry.debug("Align TX: " + vision.getTargetTx().orElse(0.0));
        telemetry.debug("Turn Power: " + turnPower);


        follower.setTeleOpDrive(yField, xField, -turnPower, true);
        double hoodPosition = ShooterConstants.MINIMUM_HOOD +(ShooterConstants.MAXIMUM_HOOD-ShooterConstants.MINIMUM_HOOD) *
                ((VisionConstants.MAXIMUM_TA-vision.getTargetTa().orElse(0.0))/(VisionConstants.MAXIMUM_TA-VisionConstants.MINIMUM_TA));

        //shooter.setHoodPosition(hoodPosition);
        telemetry.addData("Hood Setting",hoodPosition);
    }

    @Override
    public boolean isFinished() {

        return (vision.hasTarget() && turnController.atSetPoint()) || IsAprilTagNotSeemCounter>ApriltagNotSeemMaximumCounter;
    }

    @Override
    public void end(boolean interrupted) {
        follower.setTeleOpDrive(0, 0, 0, true);
        telemetry.debug("Alinhamento finalizado.");
        telemetry.update();
    }
}
