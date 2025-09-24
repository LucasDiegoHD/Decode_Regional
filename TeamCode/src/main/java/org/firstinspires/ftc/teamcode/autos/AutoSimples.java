// Ficheiro: autos/AutoShootThree.java
package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(name = "Auto: Atirar 3")
public class AutoSimples extends CommandOpMode {

    private RobotContainer robot;
    private Command autonomousCommand;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    private VisionSubsystem vision;

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new RobotContainer(hardwareMap, telemetryM, null, null);

        // Pega o NOVO comando que criamos para atirar 3
        autonomousCommand = robot.getShootThreeAutoCommand();

        // Agenda o comando para ser executado após o START
        schedule(autonomousCommand);
    }
}