package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;

/**
 * This is the main entry point for the autonomous period.
 * Its sole responsibility is to initialize the RobotContainer and schedule
 * the autonomous command selected within it.
 */
@Autonomous(name = "Autonomous Principal", group = "Competition")
public class AutoOp extends CommandOpMode {

    private RobotContainer robot;
    private Command autonomousCommand;

    @Override
    public void initialize() {
        // Initialize the telemetry manager.
        TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize the robot container. This creates all subsystems and sets up dependencies.
        robot = new RobotContainer(hardwareMap, telemetryManager, null, null);

        // Get the selected autonomous command from the robot container.
        // The logic for which auto to run is handled inside the RobotContainer.
        autonomousCommand = robot.getAutonomousCommand(telemetryManager);

        // Schedule the command to run when the OpMode starts.
        if (autonomousCommand != null) {
            schedule(autonomousCommand);
        }
    }
}

