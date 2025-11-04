package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Autonomo geral")
public class Autos extends SelectableOpMode {
    public Autos() {
        super("Selecione um auto", l -> {
            l.add("Azul Triangulo Pequeno", AutoBlueRear::new);
            l.add("Vermelho Triangulo Pequeno", AutoRedRear::new);


        });
    }

    @Override
    public void onSelect() {

    }

    @Override
    public void onLog(List<String> lines) {
    }


}