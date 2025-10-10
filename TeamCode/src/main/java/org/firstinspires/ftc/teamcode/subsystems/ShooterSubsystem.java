package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Base64;

@Configurable
public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx rshooterMotor;
    private final DcMotorEx lshooterMotor;

    private final Servo hoodServo;
    private final PIDFController pidfController;
    private final TelemetryManager telemetry;
    private double targetVelocity = 0.0;

    public ShooterSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        rshooterMotor = hardwareMap.get(DcMotorEx.class, Constants.Shooter.RSHOOTER_MOTOR_NAME);
        lshooterMotor = hardwareMap.get(DcMotorEx.class, Constants.Shooter.LSHOOTER_MOTOR_NAME);
        hoodServo = hardwareMap.get(Servo.class, Constants.Shooter.HOOD_SERVO_NAME);
        rshooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rshooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rshooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lshooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lshooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lshooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rshooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pidfController = new PIDFController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD, Constants.Shooter.kF);

        // Define uma posição inicial segura para o servo.
        setHoodPosition(Constants.Shooter.ShooterHood.RETRACTED_POSITION);
    }

    /**
     * Define a posição do servo do capuz do shooter.
     * @param position O valor do servo entre 0.0 e 1.0.
     */
    public void setHoodPosition(double position) {
        position = Math.max(Math.min(position,MAX_HOOD_POSITION),MIN_HOOD_POSITION);
        hoodServo.setPosition(position);
    }
    public void increaseHoodPosition(){
        setHoodPosition(hoodServo.getPosition()+0.05);

    }
    public void decreaseHoodPosition(){
        setHoodPosition(hoodServo.getPosition()-0.05);

    }
    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
    }

    public void stop() {
        rshooterMotor.setPower(0);
        lshooterMotor.setPower(0);
    }

    public boolean atTargetVelocity(double tolerance) {
        return Math.abs(getCurrentVelocity() - targetVelocity) < tolerance;
    }

    public double getCurrentVelocity() {
        return rshooterMotor.getVelocity();
    }

    @Override
    public void periodic() {
        /*pidfController.setPIDF(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD, Constants.Shooter.kF);

        if (targetVelocity != 0) {
            double power = pidfController.calculate(getCurrentVelocity(), targetVelocity);
            shooterMotor.setPower(power);
        } else {
            shooterMotor.setPower(0);
        }
*/
        telemetry.addData("Shooter L Velocity", lshooterMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("Shooter L Current",lshooterMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Shooter R Velocity", rshooterMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("Shooter R Current",rshooterMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Hood Position", hoodServo.getPosition());
    }

    // --- FÁBRICAS DE COMANDOS ---
    public Command spinUpCommand() {
        return new ShootCommand(this, ShootCommand.Action.SPIN_UP, Constants.Shooter.TARGET_VELOCITY);
    }

    public Command stopCommand() {
        return new ShootCommand(this, ShootCommand.Action.STOP);
    }

    public void spin() {
        rshooterMotor.setPower(1.0);
        lshooterMotor.setPower(1.0);
    }
}