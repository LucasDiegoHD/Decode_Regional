// Ficheiro: autos/AutoShootThree.java
package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotContainer;

@Autonomous(name = "Auto: Vermelho Triangulo pequeno")
public class AutoRedRear extends CommandOpMode {

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        RobotContainer robot = new RobotContainer(hardwareMap, telemetryM, null, null);

        // Pega o NOVO comando que criamos para atirar 3
        Command autonomousCommand = robot.getAutonomousRedRearCommand();
        // Agenda o comando para ser executado após o START
        schedule(autonomousCommand);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetryM.update();

    }
}