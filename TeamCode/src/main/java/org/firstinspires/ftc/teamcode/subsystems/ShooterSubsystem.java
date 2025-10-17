package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx rShooterMotor;
    private final DcMotorEx lShooterMotor;
    private final VoltageSensor voltageSensor;
    private final TelemetryManager telemetry;
    private final Servo hoodServo;

    private double targetRPM = 0.0;
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private long lastTime = System.nanoTime();

    // Valores de posição do hood
    private double hoodPosition = 0.5; // posição inicial (0.0 - 1.0)

    public ShooterSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        rShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.RSHOOTER_MOTOR_NAME);
        lShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.LSHOOTER_MOTOR_NAME);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        hoodServo = hardwareMap.get(Servo.class, ShooterConstants.HOOD_SERVO_NAME);
        rShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        rShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Inicializa o hood na posição padrão
        hoodServo.setPosition(hoodPosition);
    }

    /** Define a velocidade alvo do shooter em RPM */
    public void setTargetVelocity(double rpm) {
        targetRPM = Math.max(0, rpm);
        integralSum = 0; // evita acúmulo quando muda o setpoint
        lShooterMotor.setVelocity(RPMToTicks(rpm));
        rShooterMotor.setVelocity(RPMToTicks(rpm));
    }

    /** Para completamente o shooter */
    public void stop() {
        targetRPM = 0;
        rShooterMotor.setPower(0);
        lShooterMotor.setPower(0);
        lShooterMotor.setVelocity(0);
        rShooterMotor.setVelocity(0);
    }

    /** Aumenta o ângulo do hood */
    public void increaseHood() {
        hoodPosition = Math.min(1.0, hoodPosition + ShooterConstants.HOOD_INCREMENT);
        hoodServo.setPosition(hoodPosition);
    }

    /** Diminui o ângulo do hood */
    public void decreaseHood() {
        hoodPosition = Math.max(0.0, hoodPosition - ShooterConstants.HOOD_INCREMENT);
        hoodServo.setPosition(hoodPosition);
    }

    /** Obtém o RPM médio atual dos motores do shooter */
    private double getCurrentRPM() {
        double ticksPerSecond = (rShooterMotor.getVelocity() + lShooterMotor.getVelocity()) / 2.0;
        return (ticksPerSecond / ShooterConstants.TICKS_PER_REV) * 60.0;
    }
    private int RPMToTicks(double rpm ){
        return (int) ((rpm/60.0)*ShooterConstants.TICKS_PER_REV);
    }
    public boolean getShooterAtTarget(){
        return Math.abs(getCurrentRPM() - targetRPM) < ShooterConstants.VELOCITY_TOLERANCE;
    }


    @Override
    public void periodic() {
        telemetry.addData("Shooter at target", getShooterAtTarget());
        telemetry.addData("Shooter Target RPM", targetRPM);
        telemetry.addData("Shooter Current RPM", getCurrentRPM());
        telemetry.addData("Shooter Error", targetRPM - getCurrentRPM());
        telemetry.addData("Battery Voltage", voltageSensor.getVoltage());
        telemetry.addData("L Motor Current", lShooterMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("R Motor Current", rShooterMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Hood Position", hoodPosition);
    }
}
