package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels

public class AutoBlueRear  extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain GoToShoot1;
        public PathChain GoToLine1;
        public PathChain CatchLine1;
        public PathChain GoToShoot2;
        public PathChain GoToLine2;
        public PathChain CatchLine2;
        public PathChain GoToShoot3;
        public PathChain GoToLine3;
        public PathChain CatchLine3;
        public PathChain GoToShoot4;
        public List<PathChain> paths = new ArrayList<>();
        public Paths(Follower follower) {
            GoToShoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(65.789, 6.824), new Pose(58.615, 7.874))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(100))
                    .build();
            paths.add(GoToShoot1);
            GoToLine1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.615, 7.874), new Pose(55.640, 35.869))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(100), Math.toRadians(0))
                    .build();
            paths.add(GoToLine1);
            CatchLine1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.640, 35.869), new Pose(7.524, 34.994))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
            paths.add(CatchLine1);
            GoToShoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(7.524, 34.994), new Pose(62.989, 6.299))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(100))
                    .build();
            paths.add(GoToShoot2);
            GoToLine2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(62.989, 6.299), new Pose(45.667, 60.365))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(100), Math.toRadians(0))
                    .build();
            paths.add(GoToLine2);
            CatchLine2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.667, 60.365), new Pose(5.774, 57.565))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
            paths.add(CatchLine2);
            GoToShoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(5.774, 57.565), new Pose(57.740, 4.374))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(100))
                    .build();
            paths.add(GoToShoot3);
            GoToLine3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.740, 4.374), new Pose(40.068, 84.335))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(100), Math.toRadians(0))
                    .build();
            paths.add(GoToLine3);
            CatchLine3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.068, 84.335), new Pose(13.998, 84.160))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
            paths.add(CatchLine3);
            GoToShoot4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.998, 84.160), new Pose(55.115, 96.233))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(125))
                    .build();
            paths.add(GoToShoot4);
        }
    }

    public int autonomousPathUpdate() {
        if(!follower.isBusy()){
            follower.followPath(paths.paths.get(pathState++));
        }
        return pathState;
    }
}