package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;

import org.opencv.core.Mat;

import java.util.List;

public class RedRearPoses {

    public static final Pose[] POSES = {
            new Pose(82.411, 8.399, Math.toRadians(90)),
            new Pose(86.335, 17.147, Math.toRadians(68)),

            new Pose(134, 23, Math.toRadians(110)),
            new Pose(134, 20, Math.toRadians(90)),
            new Pose(134, 9, Math.toRadians(90)),

            new Pose(86.335, 10, Math.toRadians(90)),
            new Pose(86.335, 17.147, Math.toRadians(68)),

            new Pose(94.134, 39.169, Math.toRadians(180)),
            new Pose(127.776, 39.169, Math.toRadians(180)),

            new Pose(86.335, 17.147, Math.toRadians(68)),


            new Pose(86.510, 17.847, Math.toRadians(80)),
            new Pose(95.883, 83.111, Math.toRadians(180)),
            new Pose(134.902, 83.635, Math.toRadians(180)),

            new Pose(106, 34, Math.toRadians(90)),

    };

    public static Pose getPose(PosesNames name) {
        return POSES[name.ordinal()];
    }

    public static List<Pose> asList() {
        return List.of(POSES);
    }
}
