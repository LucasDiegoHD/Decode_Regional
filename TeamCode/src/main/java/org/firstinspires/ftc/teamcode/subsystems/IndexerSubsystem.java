package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//@AutoLog
public class IndexerSubsystem extends SubsystemBase {

    private final DigitalChannel sensorEntry;
    private final ColorSensor sensorColor;
    private final DistanceSensor sensorDistance;
    private final TelemetryManager telemetry;

    private int pieceCount = 0;

    private final boolean lastEntryState = false;
    private final boolean lastExitState = false;

    public IndexerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, IndexerConstants.EXIT_SENSOR_NAME);

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, IndexerConstants.EXIT_SENSOR_NAME);
        sensorEntry = hardwareMap.get(DigitalChannel.class, IndexerConstants.ENTRY_SENSOR_NAME);
        sensorEntry.setMode(DigitalChannel.Mode.INPUT);


    }

    @Override
    public void periodic() {
        telemetry.addData("Distance (cm)",
                sensorDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("Entry", getEntrySensor());
        telemetry.addData("Exit", getExitSensor());
        telemetry.addData("Counter", pieceCount);

    }

    public boolean getEntrySensor() {
        return sensorEntry.getState();

    }

    public boolean getExitSensor() {
        return sensorDistance.getDistance(DistanceUnit.CM) < IndexerConstants.DISTANCE_OFFSET;
    }



    public int getPieceCount() {
        return pieceCount;
    }

    public boolean hasPieces() {
        return pieceCount > 0;
    }

    public boolean isFull() {
        return pieceCount >= IndexerConstants.MAX_PIECE_CAPACITY;
    }

    public void setPieceCount(int count) {
        this.pieceCount = count;
    }
}

