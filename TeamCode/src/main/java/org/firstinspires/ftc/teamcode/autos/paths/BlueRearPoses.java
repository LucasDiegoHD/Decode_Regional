package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;

import java.util.List;

public class BlueRearPoses {


    public static final Pose[] POSES = {

            new Pose(62.411, 8.399, Math.toRadians(90)),
            new Pose(66.335, 17.147, Math.toRadians(68)),

            new Pose(50.134, 34.169, Math.toRadians(180)),
            new Pose(10.776, 34.169, Math.toRadians(180)),
            new Pose(66.335, 17.147, Math.toRadians(68)),


            new Pose(8, 23, Math.toRadians(110)),
            new Pose(8, 6, Math.toRadians(90)),

            new Pose(38, 34, Math.toRadians(90)),

    };

    public static Pose getPose(PosesNames name) {
        return POSES[name.ordinal()];
    }
    public static List<Pose> asList() {
        return List.of(POSES);
    }


}
