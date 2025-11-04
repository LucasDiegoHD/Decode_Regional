package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IndexerConstants {
    private IndexerConstants() {}

    // Hardware map names for your presence sensors (e.g., beam break sensors)
    public static String ENTRY_SENSOR_NAME = "sensorInput";
    public static String EXIT_SENSOR_NAME = "sensor_color_distance";

    // The maximum number of game pieces the robot can hold.
    public static int MAX_PIECE_CAPACITY = 3;
    public static double DISTANCE_OFFSET = 1.5;
}
