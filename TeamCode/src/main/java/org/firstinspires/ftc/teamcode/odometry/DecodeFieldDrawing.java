package org.firstinspires.ftc.teamcode.odometry;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.config.OdometryConfig;

/*
 * FILE: DecodeFieldDrawing.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/odometry/
 *
 * PURPOSE
 *   - Render an approximation of the DECODE 2025 field onto the FTC Dashboard
 *     field view using the odometry coordinate system and tunables stored in
 *     OdometryConfig.
 */
public final class DecodeFieldDrawing {

    private static final String COLOR_GRAY = "#F2F2F2";
    private static final String COLOR_WHITE = "#FFFFFF";
    private static final String COLOR_TILE = "#202020";
    private static final String COLOR_RED = "#FF0000";
    private static final String COLOR_BLUE = "#0000FF";
    private static final String COLOR_PURPLE = "#8A2BE2";
    private static final String COLOR_GREEN = "#00AA00";

    private DecodeFieldDrawing() {}

    /** Draw the static field plus the robot pose (optional). */
    public static void drawField(TelemetryPacket packet, FieldPose pose) {
        Canvas c = packet.fieldOverlay();
        drawStatic(c);
        if (pose != null) drawPose(c, pose);
    }

    private static void drawStatic(Canvas c) {
        // Mat background
        c.setFill(COLOR_GRAY);
        c.fillRect(OdometryConfig.LEFT_FIELD_X, sy(OdometryConfig.HUMAN_WALL_Y),
                OdometryConfig.FIELD_WIDTH, OdometryConfig.FIELD_LENGTH);

        // Border
        c.setStroke(COLOR_WHITE);
        c.strokeRect(OdometryConfig.LEFT_FIELD_X, sy(OdometryConfig.HUMAN_WALL_Y),
                OdometryConfig.FIELD_WIDTH, OdometryConfig.FIELD_LENGTH);

        // Tile grid every 24"
        c.setStroke(COLOR_TILE);
        for (double x = OdometryConfig.LEFT_FIELD_X; x <= OdometryConfig.RIGHT_FIELD_X; x += 24.0) {
            c.strokeLine(x, sy(OdometryConfig.HUMAN_WALL_Y), x, sy(OdometryConfig.TARGET_WALL_Y));
        }
        for (double y = OdometryConfig.HUMAN_WALL_Y; y <= OdometryConfig.TARGET_WALL_Y; y += 24.0) {
            double ys = sy(y);
            c.strokeLine(OdometryConfig.LEFT_FIELD_X, ys, OdometryConfig.RIGHT_FIELD_X, ys);
        }

        // Launch lines
        c.setStroke(COLOR_WHITE);
        c.strokeLine(OdometryConfig.LEFT_FIELD_X, sy(OdometryConfig.HUMAN_LAUNCH_LINE_Y),
                OdometryConfig.RIGHT_FIELD_X, sy(OdometryConfig.HUMAN_LAUNCH_LINE_Y));
        c.strokeLine(OdometryConfig.LEFT_FIELD_X, sy(OdometryConfig.TARGET_LAUNCH_LINE_Y),
                OdometryConfig.RIGHT_FIELD_X, sy(OdometryConfig.TARGET_LAUNCH_LINE_Y));

        // Goals
        c.setFill(COLOR_RED);
        c.fillRect(OdometryConfig.GOAL_RED_X - 2, sy(OdometryConfig.GOAL_RED_Y) - 2, 4, 4);
        c.setFill(COLOR_BLUE);
        c.fillRect(OdometryConfig.GOAL_BLUE_X - 2, sy(OdometryConfig.GOAL_BLUE_Y) - 2, 4, 4);

        // Classifiers
        c.setStroke(COLOR_RED);
        c.strokeRect(OdometryConfig.CLASSIFIER_RED_X - 3, sy(OdometryConfig.CLASSIFIER_RED_Y) - 3, 6, 6);
        c.setStroke(COLOR_BLUE);
        c.strokeRect(OdometryConfig.CLASSIFIER_BLUE_X - 3, sy(OdometryConfig.CLASSIFIER_BLUE_Y) - 3, 6, 6);

        // Base zones
        double baseHalf = OdometryConfig.BASE_ZONE_SIZE / 2.0;
        c.setStroke(COLOR_RED);
        c.strokeRect(OdometryConfig.BASE_ZONE_RED_CENTER_X - baseHalf, sy(OdometryConfig.BASE_ZONE_RED_CENTER_Y) - baseHalf,
                OdometryConfig.BASE_ZONE_SIZE, OdometryConfig.BASE_ZONE_SIZE);
        c.setStroke(COLOR_BLUE);
        c.strokeRect(OdometryConfig.BASE_ZONE_BLUE_CENTER_X - baseHalf, sy(OdometryConfig.BASE_ZONE_BLUE_CENTER_Y) - baseHalf,
                OdometryConfig.BASE_ZONE_SIZE, OdometryConfig.BASE_ZONE_SIZE);

        // Loading zones
        double loadHalf = OdometryConfig.LOADING_ZONE_SIZE / 2.0;
        c.setStroke(COLOR_WHITE);
        c.strokeRect(OdometryConfig.LOADING_ZONE_RED_CENTER_X - loadHalf, sy(OdometryConfig.LOADING_ZONE_RED_CENTER_Y) - loadHalf,
                OdometryConfig.LOADING_ZONE_SIZE, OdometryConfig.LOADING_ZONE_SIZE);
        c.strokeRect(OdometryConfig.LOADING_ZONE_BLUE_CENTER_X - loadHalf, sy(OdometryConfig.LOADING_ZONE_BLUE_CENTER_Y) - loadHalf,
                OdometryConfig.LOADING_ZONE_SIZE, OdometryConfig.LOADING_ZONE_SIZE);

        // Gates
        c.setStroke(COLOR_RED);
        c.strokeLine(OdometryConfig.GATE_RED_X - 4, sy(OdometryConfig.GATE_RED_Y), OdometryConfig.GATE_RED_X + 4, sy(OdometryConfig.GATE_RED_Y));
        c.setStroke(COLOR_BLUE);
        c.strokeLine(OdometryConfig.GATE_BLUE_X - 4, sy(OdometryConfig.GATE_BLUE_Y), OdometryConfig.GATE_BLUE_X + 4, sy(OdometryConfig.GATE_BLUE_Y));

        // Gate zones
        double gateHalfW = OdometryConfig.GATE_ZONE_WIDTH / 2.0;
        double gateHalfD = OdometryConfig.GATE_ZONE_DEPTH / 2.0;
        c.setFill("rgba(255,0,0,0.2)");
        c.fillRect(OdometryConfig.GATE_ZONE_RED_CENTER_X - gateHalfW, sy(OdometryConfig.GATE_ZONE_RED_CENTER_Y) - gateHalfD,
                OdometryConfig.GATE_ZONE_WIDTH, OdometryConfig.GATE_ZONE_DEPTH);
        c.setFill("rgba(0,0,255,0.2)");
        c.fillRect(OdometryConfig.GATE_ZONE_BLUE_CENTER_X - gateHalfW, sy(OdometryConfig.GATE_ZONE_BLUE_CENTER_Y) - gateHalfD,
                OdometryConfig.GATE_ZONE_WIDTH, OdometryConfig.GATE_ZONE_DEPTH);

        // Secret tunnels
        c.setFill("rgba(255,0,0,0.35)");
        c.fillRect(OdometryConfig.SECRET_TUNNEL_RED_X - 1, sy(OdometryConfig.SECRET_TUNNEL_RED_Y1),
                2, OdometryConfig.SECRET_TUNNEL_RED_Y2 - OdometryConfig.SECRET_TUNNEL_RED_Y1);
        c.setFill("rgba(0,0,255,0.35)");
        c.fillRect(OdometryConfig.SECRET_TUNNEL_BLUE_X - 1, sy(OdometryConfig.SECRET_TUNNEL_BLUE_Y1),
                2, OdometryConfig.SECRET_TUNNEL_BLUE_Y2 - OdometryConfig.SECRET_TUNNEL_BLUE_Y1);

        // Artifact rows (PGP ordering left→right when looking toward targets)
        drawArtifactRow(c, OdometryConfig.GPP_CENTER_X, OdometryConfig.ARTIFACT_ROW_Y,
                new String[]{"G", "P", "P"});
        drawArtifactRow(c, OdometryConfig.PGP_CENTER_X, OdometryConfig.ARTIFACT_ROW_Y,
                new String[]{"P", "G", "P"});
        drawArtifactRow(c, OdometryConfig.PPG_CENTER_X, OdometryConfig.ARTIFACT_ROW_Y,
                new String[]{"P", "P", "G"});
    }

    private static void drawArtifactRow(Canvas c, double centerX, double y, String[] order) {
        double startX = centerX - OdometryConfig.ARTIFACT_SPACING;
        for (int i = 0; i < order.length; i++) {
            String color = order[i].equals("G") ? COLOR_GREEN : COLOR_PURPLE;
            c.setFill(color);
            c.fillCircle(startX + i * OdometryConfig.ARTIFACT_SPACING, sy(y), 1.75);
        }
    }

    private static void drawPose(Canvas c, FieldPose pose) {
        c.setFill("#333333");
        c.fillCircle(pose.x, sy(pose.y), 3.0);
        double headingRad = Math.toRadians(pose.headingDeg);
        double hx = pose.x + 8.0 * Math.sin(headingRad);
        double hy = sy(pose.y + 8.0 * Math.cos(headingRad));
        c.setStroke("#000000");
        c.strokeLine(pose.x, sy(pose.y), hx, hy);
    }

    private static double sy(double fieldY) {
        return fieldY - OdometryConfig.FIELD_LENGTH / 2.0;
    }
}
