// Ficheiro: autos/AutoShootThree.java
package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.telemetry.SelectScope;
import com.pedropathing.telemetry.Selector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotContainer;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

@Autonomous(name = "Autonomo geral")
public class Autos extends CommandOpMode {
    private Command selectedCommand;
    private final static String[] MESSAGE = {
            "Use as setas para mover o cursor.",
            "Pressione right bumper para select.",
            "Pressione left bumper para voltar."
    };
    @IgnoreConfigurable
    static TelemetryManager telemetryM;


    public void onSelect() {
        schedule(selectedCommand);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void initialize() {
        RobotContainer robot = new RobotContainer(hardwareMap, telemetryM, null, null);

        Consumer<SelectScope<Supplier<Command>>> autosToSelect = s -> {
            s.add("Azul Triangulo pequeno", robot::getAutonomousBlueRearCommand);
            s.add("Vermelho Triangulo pequeno", robot::getAutonomousRedRearCommand);
        };
        Selector<Supplier<Command>> selector = Selector.create("Autonomos", autosToSelect, MESSAGE);
        selector.onSelect(opModeSupplier -> {
            onSelect();
            selectedCommand = opModeSupplier.get();

        });

        while (opModeInInit()) {
            if (selectedCommand == null) {
                if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed())
                    selector.decrementSelected();
                else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed())
                    selector.incrementSelected();
                else if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed())
                    selector.select();
                else if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed())
                    selector.goBack();

                List<String> lines = selector.getLines();
                for (String line : lines) {
                    telemetry.addLine(line);
                }
            }

        }

        // Pega o NOVO comando que criamos para atirar 3
        // Agenda o comando para ser executado após o START

    }


    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetryM.update();

    }
}