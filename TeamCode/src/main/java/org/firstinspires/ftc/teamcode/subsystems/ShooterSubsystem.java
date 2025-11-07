package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


/**
 * The ShooterSubsystem is responsible for controlling the robot's shooting mechanism.
 * This includes managing the speed of the shooter motors and the angle of the hood.
 */
/// /@AutoLog

public class ShooterSubsystem extends SubsystemBase {
    /*

        formula para calcular o RPM pela distancia
        RPM = 520.71 * distancia + 3815.97
     */
    private final DcMotorEx rShooterMotor;
    private final DcMotorEx lShooterMotor;
    private final VoltageSensor voltageSensor;
    private final TelemetryManager telemetry;
    private final Servo hoodServo;

    private double targetRPM = 0.0;

    // Hood position values
    private double hoodPosition = 0.5; // initial position (0.0 - 1.0)

    /**
     * Constructs a new ShooterSubsystem.
     *
     * @param hardwareMap The hardware map to retrieve hardware devices from.
     * @param telemetry   The telemetry manager for logging.
     */
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

        // Initialize hood to default position
        hoodServo.setPosition(ShooterConstants.MAXIMUM_HOOD);
    }

    public void setHoodPosition(double position) {
        hoodPosition = position;
        hoodServo.setPosition(hoodPosition);
    }
    /**
     * Sets the target velocity of the shooter in RPM.
     * @param rpm The target RPM.
     */
    public void setTargetVelocity(double rpm) {
        targetRPM = Math.max(0, rpm);
        lShooterMotor.setVelocity(RPMToTicks(rpm));
        rShooterMotor.setVelocity(RPMToTicks(rpm));
    }

    /**
     * Completely stops the shooter.
     */
    public void stop() {
        targetRPM = 0;

        lShooterMotor.setVelocity(0);
        rShooterMotor.setVelocity(0);
    }

    /**
     * Increases the angle of the hood.
     */
    public void increaseHood() {
        hoodPosition = Math.min(ShooterConstants.MAXIMUM_HOOD, hoodPosition + ShooterConstants.HOOD_INCREMENT);
        hoodServo.setPosition(hoodPosition);
    }

    /**
     * Decreases the angle of the hood.
     */
    public void decreaseHood() {
        hoodPosition = Math.max(ShooterConstants.MINIMUM_HOOD, hoodPosition - ShooterConstants.HOOD_INCREMENT);
        hoodServo.setPosition(hoodPosition);
    }

    /**
     * Gets the current average RPM of the shooter motors.
     * @return The current RPM.
     */
    protected double getCurrentRPM() {
        double ticksPerSecond = (rShooterMotor.getVelocity() + lShooterMotor.getVelocity()) / 2.0;
        return (ticksPerSecond / ShooterConstants.TICKS_PER_REV) * 60.0;
    }

    /**
     * Converts RPM to ticks per second.
     * @param rpm The RPM to convert.
     * @return The equivalent ticks per second.
     */
    protected int RPMToTicks(double rpm) {
        return (int) ((rpm / 60.0) * ShooterConstants.TICKS_PER_REV);
    }

    /**
     * Checks if the shooter is at its target velocity.
     * @return True if the current RPM is within the tolerance of the target RPM, false otherwise.
     */
    public boolean getShooterAtTarget() {
        return Math.abs(getCurrentRPM() - targetRPM) < ShooterConstants.VELOCITY_TOLERANCE;
    }


    /**
     * This method is called periodically to update the subsystem's state and telemetry.
     */
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