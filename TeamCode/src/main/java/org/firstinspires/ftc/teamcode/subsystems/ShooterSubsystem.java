package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@Configurable
public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx rshooterMotor;
    private final DcMotorEx lshooterMotor;
    private VoltageSensor myControlHubVoltageSensor;

    private final Servo hoodServo;
    private final PIDFController pidfController;
    private final TelemetryManager telemetry;
    private double targetVelocity = 0.0;

    private double voltage = 0.0;

    public ShooterSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        rshooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.RSHOOTER_MOTOR_NAME);
        lshooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.LSHOOTER_MOTOR_NAME);
        hoodServo = hardwareMap.get(Servo.class, ShooterConstants.HOOD_SERVO_NAME);
        rshooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rshooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rshooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lshooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lshooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lshooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rshooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        pidfController = new PIDFController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);

        setHoodPosition(ShooterConstants.MAXIMUM_HOOD);
    }

    /**
     * Define a posição do servo do capuz do shooter.
     * @param position O valor do servo entre 0.0 e 1.0.
     */
    public void setHoodPosition(double position) {
        position = Math.max(Math.min(position,ShooterConstants.MAXIMUM_HOOD),ShooterConstants.MINIMUM_HOOD);
        hoodServo.setPosition(position);
    }
    public double getBatteryVoltage() {
        return myControlHubVoltageSensor.getVoltage();

    }
    private void reloadShooterVoltage(){
        double power = (voltage+0.9)/getBatteryVoltage();
        lshooterMotor.setPower(power);
        rshooterMotor.setPower(power);
    }
    public void setShooterVoltage(double voltage){
        this.voltage = voltage;

    }
    public void increaseHoodPosition(){
        setHoodPosition(hoodServo.getPosition()+0.1);

    }
    public void decreaseHoodPosition(){
        setHoodPosition(hoodServo.getPosition()-0.1);

    }
    public void setTargetVelocity() {
        this.targetVelocity = ShooterConstants.TARGET_VELOCITY;
    }

    public void stop() {
        targetVelocity = 0;
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
        pidfController.setPIDF(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);

        if (targetVelocity != 0) {
            double power = pidfController.calculate(getCurrentVelocity(), targetVelocity);
            lshooterMotor.setPower(power);
            rshooterMotor.setPower(power);
        } else {
            lshooterMotor.setPower(0);
            rshooterMotor.setPower(0);
        }
        telemetry.addData("Shooter L Velocity", ticksPerSecondToRPM(lshooterMotor.getVelocity()));
        telemetry.addData("Shooter L Current",lshooterMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Shooter R Velocity", ticksPerSecondToRPM(rshooterMotor.getVelocity()));
        telemetry.addData("Shooter R Current",rshooterMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Hood Position", hoodServo.getPosition());
        telemetry.addData("Battery Voltage", getBatteryVoltage());
    }

    // --- FÁBRICAS DE COMANDOS ---
    public Command spinUpCommand() {
        return new ShootCommand(this, ShootCommand.Action.SPIN_UP, ShooterConstants.TARGET_VELOCITY);
    }
    double radiansPerSecondToRPM(double rps){
        return rps * 60/(2*Math.PI);
    }
    double ticksPerSecondToRPM(double tps){
        return (tps/28)*60;
    }
    public Command stopCommand() {
        return new ShootCommand(this, ShootCommand.Action.STOP);
    }

    public void spin() {
        rshooterMotor.setPower(0.75);
        lshooterMotor.setPower(0.75);
    }
}