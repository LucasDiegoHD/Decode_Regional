package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
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

    private double hoodPosition = 0.5;
    private static final double HOOD_INCREMENT = 0.02;

    public ShooterSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        rShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.RSHOOTER_MOTOR_NAME);
        lShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.LSHOOTER_MOTOR_NAME);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        hoodServo = hardwareMap.get(Servo.class, ShooterConstants.HOOD_SERVO_NAME);

        rShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo.setPosition(hoodPosition);
    }

    public void setTargetVelocity(double rpm) {
        targetRPM = Math.max(0, rpm);
        integralSum = 0;
    }

    public void stop() {
        targetRPM = 0;
    }

    public void increaseHood() {
        hoodPosition = Math.min(ShooterConstants.MAXIMUM_HOOD, hoodPosition + HOOD_INCREMENT);
        hoodServo.setPosition(hoodPosition);
    }

    public void decreaseHood() {
        hoodPosition = Math.max(ShooterConstants.MINIMUM_HOOD, hoodPosition - HOOD_INCREMENT);
        hoodServo.setPosition(hoodPosition);
    }

    /**
     * ADICIONADO: Define a posição do hood para um valor absoluto.
     * Essencial para a mira automática.
     * @param position A posição do servo (0.0 a 1.0), que será limitada pelos valores MIN/MAX.
     */
    public void setHoodPosition(double position) {
        hoodPosition = Math.max(ShooterConstants.MINIMUM_HOOD, Math.min(ShooterConstants.MAXIMUM_HOOD, position));
        hoodServo.setPosition(hoodPosition);
    }

    /**
     * ADICIONADO: Expõe o RPM alvo atual.
     * Essencial para o OpMode de afinação e para a telemetria.
     * @return O RPM alvo atual.
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * ADICIONADO: Verifica se o shooter atingiu a velocidade alvo.
     * Essencial para os comandos de autônomo saberem quando podem lançar.
     * @return true se a velocidade atual estiver dentro da tolerância definida.
     */
    public boolean atTargetVelocity() {
        return Math.abs(getCurrentRPM() - targetRPM) < ShooterConstants.VELOCITY_TOLERANCE;
    }

    private double getCurrentRPM() {
        double ticksPerSecond = (rShooterMotor.getVelocity() + lShooterMotor.getVelocity()) / 2.0;
        return (ticksPerSecond / ShooterConstants.TICKS_PER_REV) * 60.0;
    }

    private double pidfCalculate(double currentRPM) {
        double error = targetRPM - currentRPM;
        double deltaTime = (System.nanoTime() - lastTime) / 1e9;
        lastTime = System.nanoTime();

        if (deltaTime <= 0) deltaTime = 1e-3;

        integralSum += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        double pTerm = ShooterConstants.kP * error;
        double iTerm = ShooterConstants.kI * integralSum;
        double dTerm = ShooterConstants.kD * derivative;

        double voltageComp = 12.0 / voltageSensor.getVoltage();
        double fTerm = ShooterConstants.kF * targetRPM * voltageComp;

        double output = pTerm + iTerm + dTerm + fTerm;
        return Math.max(0, output);
    }

    @Override
    public void periodic() {
        double currentRPM = getCurrentRPM();
        double power = 0;

        if (targetRPM > 0) {
            power = pidfCalculate(currentRPM);
            rShooterMotor.setPower(power);
            lShooterMotor.setPower(power);
        } else {
            rShooterMotor.setPower(0);
            lShooterMotor.setPower(0);
        }

        telemetry.addData("Shooter Target RPM", targetRPM);
        telemetry.addData("Shooter Current RPM", currentRPM);
        telemetry.addData("Shooter Power", power);
        telemetry.addData("Shooter Error", targetRPM - currentRPM);
        telemetry.addData("Battery Voltage", voltageSensor.getVoltage());
        telemetry.addData("L Motor Current", lShooterMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("R Motor Current", rShooterMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Hood Position", hoodPosition);
    }
}

