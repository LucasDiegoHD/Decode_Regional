package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

/**
 * Comando de condução TeleOp, delega toda validação para o DrivetrainSubsystem.
 * Agora também loga os valores dos joysticks para debug.
 */
public class TeleOpDriveCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final GamepadEx driver;
    private boolean isFieldCentric = false; // Começa sempre no modo seguro

    public TeleOpDriveCommand(DrivetrainSubsystem drivetrain, GamepadEx driver) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        addRequirements(drivetrain);
    }

    /**
     * Método público para alternar o modo (chamado pelo RobotContainer).
     */
    public void toggleDriveMode() {
        isFieldCentric = !isFieldCentric;
    }

    @Override
    public void execute() {
        // Lê os valores dos joysticks (com fallback 0.0 se driver for null)
        double leftY = 0.0;
        double leftX = 0.0;
        double rightX = 0.0;

        if (driver != null) {
            try {
                leftY = driver.getLeftY();
                leftX = driver.getLeftX();
                rightX = driver.getRightX();
            } catch (Exception e) {
                // se algo der errado ao ler o gamepad, registrar e seguir com zeros
                drivetrain.debug("Erro ao ler GamepadEx: " + e.getMessage());
            }
        } else {
            drivetrain.debug("TeleOpDriveCommand: driver GamepadEx é null.");
        }

        // Log dos valores para ajudar a depuração (verifique Driver Hub)
        drivetrain.debug(String.format("Inputs joysticks -> LY=%.3f, LX=%.3f, RX=%.3f, FieldCentric=%b",
                leftY, leftX, rightX, true));

        // Chama o método seguro do subsistema (ele fará checagens e evitará NPEs)
        drivetrain.teleOpDriveSafe(-leftY, leftX, -rightX, isFieldCentric);
    }
}
