package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;

import java.util.List;

public class BlueRearPoses {


    public static final Pose[] POSES = {
            new Pose(65.789, 6.824, Math.toRadians(90)),
            new Pose(58.615, 7.874, Math.toRadians(100)),

            new Pose(55.640, 35.869, Math.toRadians(0)),
            new Pose(7.524, 34.994, Math.toRadians(0)),

            new Pose(62.989, 6.299, Math.toRadians(100)),
            new Pose(45.667, 60.365, Math.toRadians(0)),
            new Pose(5.774, 57.565, Math.toRadians(0)),

            new Pose(57.740, 4.374, Math.toRadians(100)),
            new Pose(40.068, 84.335, Math.toRadians(0)),
            new Pose(13.998, 84.160, Math.toRadians(0)),

            new Pose(55.115, 96.233, Math.toRadians(125))
    };

    public static Pose getPose(PosesNames name) {
        return POSES[name.ordinal()];
    }
    public static List<Pose> asList() {
        return List.of(POSES);
    }


}
