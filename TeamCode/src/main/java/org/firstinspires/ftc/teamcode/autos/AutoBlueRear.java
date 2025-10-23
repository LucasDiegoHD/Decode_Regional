package org.firstinspires.ftc.teamcode.autos;

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
@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels

public class AutoBlueRear  extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
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

        public PathChain GoToLine1;
        public PathChain CatchLine1;
        public PathChain GoToShoot2;
        public PathChain GoToLine2;
        public PathChain CatchLine2;
        public PathChain GoToShoot3;
        public PathChain GoToLine3;
        public PathChain CatchLine3;
        public PathChain GoToShoot4;

        public Paths(Follower follower) {
            GoToLine1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(55.640, 35.869))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            CatchLine1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.640, 35.869), new Pose(7.524, 34.994))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            GoToShoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(7.524, 34.994), new Pose(60.190, 6.474))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(100))
                    .build();

            GoToLine2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.190, 6.474), new Pose(45.667, 60.365))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(100), Math.toRadians(0))
                    .build();

            CatchLine2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.667, 60.365), new Pose(5.774, 57.565))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            GoToShoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(5.774, 57.565), new Pose(60.714, 5.599))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(100))
                    .build();

            GoToLine3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.714, 5.599), new Pose(40.068, 84.335))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(100), Math.toRadians(0))
                    .build();

            CatchLine3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.068, 84.335), new Pose(13.998, 84.160))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            GoToShoot4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.998, 84.160), new Pose(55.115, 96.233))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(125))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}