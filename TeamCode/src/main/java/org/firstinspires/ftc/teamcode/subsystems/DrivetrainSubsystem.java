package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    private final Follower follower;
    private final TelemetryManager telemetry;
    private final VisionSubsystem vision; // ADICIONADO: Dependência da Visão
    private final IMU imu;               // ADICIONADO: Dependência da IMU

    // ALTERADO: O construtor agora aceita as novas dependências
    public DrivetrainSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry, VisionSubsystem vision, IMU imu) {
        this.follower = Constants.Drivetrain.createFollower(hardwareMap);
        this.telemetry = telemetry;
        this.vision = vision;
        this.imu = imu;
        Drawing.init();
    }

    public Follower getFollower() {
        return follower;
    }

    @Override
    public void periodic() {
        // 1. A sua lógica original de atualização da odometria (Pinpoint)
        follower.update();

        // --- INÍCIO DA NOVA LÓGICA DE FUSÃO DE SENSORES ---
        // 2. Obtém a orientação absoluta e estável da IMU.
        double currentHeadingRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // 3. Tenta obter uma correção de posição (X, Y) da Limelight,
        //    passando a orientação da IMU para ser usada na Pose final.
        vision.getRobotPose(currentHeadingRadians).ifPresent(visionPose -> {
            // 4. Se a visão deu uma pose válida, atualiza a pose do Follower.
            //    Isto corrige o drift da odometria das rodas.
            getFollower().setPose(visionPose);
        });
        // --- FIM DA NOVA LÓGICA ---

        // 5. A sua lógica original de telemetria e desenho permanece intacta.
        telemetry.addData("Robot pose", follower.getPose());
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();
    }
}

/**
 * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
 *
 * @author Lazar - 19234
 * @version 1.1, 5/19/2025
 */
class Drawing {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.0
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.0
    );

    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);
        sendPacket();
    }

    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }
        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);
        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();
        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();
        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }
        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);
        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {
            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    public static void sendPacket() {
        panelsField.update();
    }
}

