package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, Constants.Intake.INTAKE_MOTOR);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // --- Ações do Subsistema ---
    public void run() { intakeMotor.setPower(1.0); }
    public void feed() { intakeMotor.setPower(-0.8); } // Alimentar é o inverso de coletar
    public void stop() { intakeMotor.setPower(0); }

    /**
     * Placeholder para lógica futura com sensor.
     * @return true se uma peça de jogo for detetada.
     */
    public boolean hasGamePiece() {
        // Lógica do sensor (ex: sensor de cor, sensor de distância) viria aqui
        return false;
    }

    // --- FÁBRICAS DE COMANDOS ---
    // Isto permite-nos criar comandos de forma limpa no RobotContainer
    public Command intakeCommand() {
        return new InstantCommand(this::run, this);
    }

    public Command feedCommand() {
        return new InstantCommand(this::feed, this);
    }

    public Command stopCommand() {
        return new InstantCommand(this::stop, this);
    }
}
