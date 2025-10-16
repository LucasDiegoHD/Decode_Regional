package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;

@Autonomous(name = "Autonomous Principal", group = "Competition")
public class AutoOp extends CommandOpMode {

    private RobotContainer robot;
    private Command autonomousCommand;

    @Override
    public void initialize() {
        TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize the robot container. This creates all subsystems.
        robot = new RobotContainer(hardwareMap, telemetryManager, null, null);

        // Get the selected autonomous command from the robot container.
        autonomousCommand = robot.getAutonomousCommand(telemetryManager, null);

        // Schedule the command to run when the OpMode starts.
        schedule(autonomousCommand);
    }
}
