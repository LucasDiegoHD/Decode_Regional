package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;

import java.util.List;

public class BlueRearPoses {


    public static final Pose[] POSES = {
            new Pose(82.411, 8.399, Math.toRadians(90)),
            new Pose(86.335, 17.147, Math.toRadians(68)),

            new Pose(94.134, 36.169, Math.toRadians(180)),
            new Pose(136.776, 36.169, Math.toRadians(180)),
            new Pose(86.335, 17.147, Math.toRadians(68)),


            new Pose(136, 20, Math.toRadians(95)),
            new Pose(136, 6, Math.toRadians(90)),
            new Pose(40, 36, Math.toRadians(90)),


    };

    public static Pose getPose(PosesNames name) {
        return POSES[name.ordinal()];
    }
    public static List<Pose> asList() {
        return List.of(POSES);
    }


}
