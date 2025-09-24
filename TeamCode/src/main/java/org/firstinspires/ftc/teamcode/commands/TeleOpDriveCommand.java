package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.Optional;

public class TeleOpDriveCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final GamepadEx driver;
    private final TelemetryManager telemetry;

    public TeleOpDriveCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, GamepadEx driver, TelemetryManager telemetry) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.driver = driver;
        this.telemetry = telemetry;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Obtém as entradas do gamepad
        double x = driver.getLeftX();
        double y = -driver.getLeftY(); // O Y no gamepad é invertido
        double rx = driver.getRightX();

        // Envia os comandos de movimento para o follower do Pedro Pathing.
        // O `false` no final indica controle robot-centric, sem usar a orientação do robô.
        drivetrain.getFollower().setTeleOpDrive(y, x, rx, false);

        // Tenta obter a pose 3D da Limelight
        Optional<Pose3D> botpose = vision.getBotpose();

        // Envia telemetria para depuração
        telemetry.addData("Vision Target Found", vision.hasTarget());
        if (botpose.isPresent()) {
            Pose3D pose = botpose.get();
            // Posição x, y, z da Limelight em relação à AprilTag.
            telemetry.addData("Botpose X (inch)", pose.getPosition().x);
            telemetry.addData("Botpose Y (inch)", pose.getPosition().y);
            telemetry.addData("Botpose Z (inch)", pose.getPosition().z);
        }
    }
}