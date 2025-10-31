package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


//@AutoLog
public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor intakeMotor;
    private final DcMotor triggerMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor"); // Use o nome da sua configuração
        triggerMotor = hardwareMap.get(DcMotor.class, "triggerMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        triggerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run() { intakeMotor.setPower(1.0); }
    public void reverse() { intakeMotor.setPower(-1.0); }
    public void stop() { intakeMotor.setPower(0); }
    public void runTrigger() { triggerMotor.setPower(-1.0); }
    public void stopTrigger() { triggerMotor.setPower(0); }

}