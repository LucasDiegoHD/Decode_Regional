package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;

import java.util.List;

public class BlueRearPoses {


    public static final Pose[] POSES = {
            new Pose(58.411, 8.399, Math.toRadians(90)),
            new Pose(60.335, 17.147, Math.toRadians(68)),

            new Pose(70.134, 39.169, Math.toRadians(180)),
            new Pose(103.776, 39.169, Math.toRadians(180)),

            new Pose(60.335, 17.147, Math.toRadians(68)),
            new Pose(70.134, 59.840, Math.toRadians(180)),

            new Pose(117.550, 59.840, Math.toRadians(180)),
            new Pose(60.335, 17.847, Math.toRadians(80)),

            new Pose(70.134, 83.111, Math.toRadians(180)),
            new Pose(110.902, 83.635, Math.toRadians(180)),

            new Pose(54.211, 85.910, Math.toRadians(45))
    };

    public static Pose getPose(PosesNames name) {
        return POSES[name.ordinal()];
    }
    public static List<Pose> asList() {
        return List.of(POSES);
    }


}
