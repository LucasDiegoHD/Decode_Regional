package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;

import java.util.List;

public class RedRearPoses {

    public static final Pose[] POSES = {
            new Pose(82.411, 8.399, Math.toRadians(90)),
            new Pose(84.335, 17.147, Math.toRadians(68)),

            new Pose(94.134, 39.169, Math.toRadians(180)),
            new Pose(127.776, 39.169, Math.toRadians(180)),

            new Pose(84.335, 17.147, Math.toRadians(68)),
            new Pose(93.959, 59.840, Math.toRadians(180)),
            new Pose(141.550, 59.665, Math.toRadians(180)),

            new Pose(84.510, 17.847, Math.toRadians(80)),
            new Pose(95.883, 83.111, Math.toRadians(180)),
            new Pose(134.902, 83.635, Math.toRadians(180)),

            new Pose(78.211, 85.910, Math.toRadians(45))
    };

    public static Pose getPose(PosesNames name) {
        return POSES[name.ordinal()];
    }

    public static List<Pose> asList() {
        return List.of(POSES);
    }
}
