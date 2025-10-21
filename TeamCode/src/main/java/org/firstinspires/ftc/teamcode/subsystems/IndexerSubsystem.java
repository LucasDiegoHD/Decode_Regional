package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IndexerSubsystem extends SubsystemBase {

    private final DigitalChannel entrySensor;
    private final DigitalChannel exitSensor;
    private final TelemetryManager telemetry;

    private int pieceCount = 0;

    private boolean lastEntryState = false;
    private boolean lastExitState = false;

    public IndexerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        entrySensor = hardwareMap.get(DigitalChannel.class, IndexerConstants.ENTRY_SENSOR_NAME);
        exitSensor = hardwareMap.get(DigitalChannel.class, IndexerConstants.EXIT_SENSOR_NAME);

        entrySensor.setMode(DigitalChannel.Mode.INPUT);
        exitSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void periodic() {
        boolean entryTriggered = !entrySensor.getState();
        boolean exitTriggered = !exitSensor.getState();

        if (entryTriggered && !lastEntryState) {
            pieceCount++;
        }

        if (exitTriggered && !lastExitState) {
            pieceCount = Math.max(0, pieceCount - 1);
        }

        lastEntryState = entryTriggered;
        lastExitState = exitTriggered;

        // THIS IS THE NEW LINE: Send the current piece count to telemetry.
        telemetry.addData("Pieces Count", getPieceCount());
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

