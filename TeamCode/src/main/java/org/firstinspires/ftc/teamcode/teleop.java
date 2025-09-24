package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp
public class teleop extends CommandOpMode {

    private RobotContainer robot;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private Follower follower;

    private TelemetryManager telemetryM;
    private VisionSubsystem vision;

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // Passa o hardwareMap, a telemetria e os gamepads para o RobotContainer
        robot = new RobotContainer(hardwareMap, telemetryM, driverGamepad, operatorGamepad);
    }
}
