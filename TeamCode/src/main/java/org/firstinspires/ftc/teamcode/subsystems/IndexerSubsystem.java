package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * The IndexerSubsystem is responsible for managing the game pieces within the robot's indexer.
 * It uses a variety of sensors to detect the presence and count of game pieces.
 */
//@AutoLog
public class IndexerSubsystem extends SubsystemBase {

    private final DigitalChannel sensorEntry;
    private final DistanceSensor sensorDistance;
    private final TelemetryManager telemetry;

    private int pieceCount = 0;

    private final boolean lastEntryState = false;
    private final boolean lastExitState = false;

    /**
     * Constructs a new IndexerSubsystem.
     *
     * @param hardwareMap The hardware map to retrieve hardware devices from.
     * @param telemetry   The telemetry manager for logging.
     */
    public IndexerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        // get a reference to the color sensor.
        ColorSensor sensorColor = hardwareMap.get(ColorSensor.class, IndexerConstants.EXIT_SENSOR_NAME);

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, IndexerConstants.EXIT_SENSOR_NAME);
        sensorEntry = hardwareMap.get(DigitalChannel.class, IndexerConstants.ENTRY_SENSOR_NAME);
        sensorEntry.setMode(DigitalChannel.Mode.INPUT);


    }

    /**
     * This method is called periodically to update the subsystem's state and telemetry.
     */
    @Override
    public void periodic() {
        telemetry.addData("Distance (cm)",
                sensorDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Entry", getEntrySensor());
        telemetry.addData("Exit", getExitSensor());

    }

    /**
     * Gets the state of the entry sensor.
     * @return True if the sensor is triggered, false otherwise.
     */
    public boolean getEntrySensor() {
        return sensorEntry.getState();

    }

    /**
     * Gets the state of the exit sensor.
     * @return True if the sensor is triggered, false otherwise.
     */
    public boolean getExitSensor() {
        return sensorDistance.getDistance(DistanceUnit.CM) < IndexerConstants.DISTANCE_OFFSET;
    }


    /**
     * Gets the current number of game pieces in the indexer.
     * @return The number of game pieces.
     */
    public int getPieceCount() {
        return pieceCount;
    }

    /**
     * Checks if the indexer has any game pieces.
     * @return True if the indexer contains at least one piece, false otherwise.
     */
    public boolean hasPieces() {
        return pieceCount > 0;
    }

    /**
     * Checks if the indexer is full.
     * @return True if the indexer is at maximum capacity, false otherwise.
     */
    public boolean isFull() {
        return pieceCount >= IndexerConstants.MAX_PIECE_CAPACITY;
    }

    /**
     * Sets the number of game pieces in the indexer.
     * @param count The new piece count.
     */
    public void setPieceCount(int count) {
        this.pieceCount = count;
    }
}
