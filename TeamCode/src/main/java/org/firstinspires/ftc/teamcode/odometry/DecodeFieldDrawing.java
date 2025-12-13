package org.firstinspires.ftc.teamcode.odometry;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.config.OdometryConfig;
import org.firstinspires.ftc.teamcode.utils.ObeliskSignal;

/*
 * FILE: DecodeFieldDrawing.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/odometry/
 *
 * PURPOSE
 *   - Render an approximation of the DECODE 2025 field onto the FTC Dashboard
 *     field view using the odometry coordinate system and tunables stored in
 *     OdometryConfig.
 *
 * CHANGES (2025-11-26): Added odometry→dashboard transform helpers, triangular
 *                        launch zones, alliance-aware artifact rows with spacing
 *                        + radius tunables, and dashboard telemetry packet
 *                        support for TeleOp/Auto callers.
 * CHANGES (2025-12-11): Updated drawing transforms for the field-center
 *                        odometry frame (no Y shift) so overlays match fused
 *                        poses with Limelight XY fusion enabled.
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
    public static void drawField(TelemetryPacket packet, FieldPose pose, Alliance alliance, ObeliskSignal.Order motif) {
        Canvas c = packet.fieldOverlay();
        drawStatic(c, motif);
        if (pose != null) {
            drawPose(c, pose);
            drawPoseText(c, pose);
        }
    }

    private static void drawStatic(Canvas c, ObeliskSignal.Order motif) {
        // Mat background
        c.setFill(COLOR_GRAY);
        c.fillRect(toDashX(OdometryConfig.LEFT_FIELD_X), toDashY(OdometryConfig.HUMAN_WALL_Y),
                OdometryConfig.FIELD_WIDTH, OdometryConfig.FIELD_LENGTH);

        // Border
        c.setStroke(COLOR_WHITE);
        c.strokeRect(toDashX(OdometryConfig.LEFT_FIELD_X), toDashY(OdometryConfig.HUMAN_WALL_Y),
                OdometryConfig.FIELD_WIDTH, OdometryConfig.FIELD_LENGTH);

        // Tile grid every 24"
        c.setStroke(COLOR_TILE);
        for (double x = OdometryConfig.LEFT_FIELD_X; x <= OdometryConfig.RIGHT_FIELD_X; x += 24.0) {
            double dashX = toDashX(x);
            c.strokeLine(dashX, toDashY(OdometryConfig.HUMAN_WALL_Y), dashX, toDashY(OdometryConfig.TARGET_WALL_Y));
        }
        for (double y = OdometryConfig.HUMAN_WALL_Y; y <= OdometryConfig.TARGET_WALL_Y; y += 24.0) {
            double ys = toDashY(y);
            c.strokeLine(toDashX(OdometryConfig.LEFT_FIELD_X), ys, toDashX(OdometryConfig.RIGHT_FIELD_X), ys);
        }

        // Launch lines
        c.setStroke(COLOR_WHITE);
        c.strokeLine(toDashX(OdometryConfig.LEFT_FIELD_X), toDashY(OdometryConfig.HUMAN_LAUNCH_LINE_Y),
                toDashX(OdometryConfig.RIGHT_FIELD_X), toDashY(OdometryConfig.HUMAN_LAUNCH_LINE_Y));
        c.strokeLine(toDashX(OdometryConfig.LEFT_FIELD_X), toDashY(OdometryConfig.TARGET_LAUNCH_LINE_Y),
                toDashX(OdometryConfig.RIGHT_FIELD_X), toDashY(OdometryConfig.TARGET_LAUNCH_LINE_Y));

        // Launch triangles
        c.setStroke(COLOR_WHITE);
        drawTriangle(c,
                OdometryConfig.HUMAN_LAUNCH_LEFT_X, OdometryConfig.HUMAN_LAUNCH_LEFT_Y,
                OdometryConfig.HUMAN_LAUNCH_RIGHT_X, OdometryConfig.HUMAN_LAUNCH_RIGHT_Y,
                OdometryConfig.HUMAN_LAUNCH_APEX_X, OdometryConfig.HUMAN_LAUNCH_APEX_Y);
        drawTriangle(c,
                OdometryConfig.TARGET_LAUNCH_LEFT_X, OdometryConfig.TARGET_LAUNCH_LEFT_Y,
                OdometryConfig.TARGET_LAUNCH_RIGHT_X, OdometryConfig.TARGET_LAUNCH_RIGHT_Y,
                OdometryConfig.TARGET_LAUNCH_APEX_X, OdometryConfig.TARGET_LAUNCH_APEX_Y);

        // Goals
        c.setFill(COLOR_RED);
        c.fillRect(toDashX(OdometryConfig.GOAL_RED_X) - 2, toDashY(OdometryConfig.GOAL_RED_Y) - 2, 4, 4);
        c.setFill(COLOR_BLUE);
        c.fillRect(toDashX(OdometryConfig.GOAL_BLUE_X) - 2, toDashY(OdometryConfig.GOAL_BLUE_Y) - 2, 4, 4);

        // Classifiers
        c.setStroke(COLOR_RED);
        c.strokeRect(toDashX(OdometryConfig.CLASSIFIER_RED_X) - 3, toDashY(OdometryConfig.CLASSIFIER_RED_Y) - 3, 6, 6);
        c.setStroke(COLOR_BLUE);
        c.strokeRect(toDashX(OdometryConfig.CLASSIFIER_BLUE_X) - 3, toDashY(OdometryConfig.CLASSIFIER_BLUE_Y) - 3, 6, 6);

        // Base zones
        double baseHalf = OdometryConfig.BASE_ZONE_SIZE / 2.0;
        c.setStroke(COLOR_RED);
        c.strokeRect(toDashX(OdometryConfig.BASE_ZONE_RED_CENTER_X) - baseHalf, toDashY(OdometryConfig.BASE_ZONE_RED_CENTER_Y) - baseHalf,
                OdometryConfig.BASE_ZONE_SIZE, OdometryConfig.BASE_ZONE_SIZE);
        c.setStroke(COLOR_BLUE);
        c.strokeRect(toDashX(OdometryConfig.BASE_ZONE_BLUE_CENTER_X) - baseHalf, toDashY(OdometryConfig.BASE_ZONE_BLUE_CENTER_Y) - baseHalf,
                OdometryConfig.BASE_ZONE_SIZE, OdometryConfig.BASE_ZONE_SIZE);

        // Loading zones
        double loadHalf = OdometryConfig.LOADING_ZONE_SIZE / 2.0;
        c.setStroke(COLOR_WHITE);
        c.strokeRect(toDashX(OdometryConfig.LOADING_ZONE_RED_CENTER_X) - loadHalf, toDashY(OdometryConfig.LOADING_ZONE_RED_CENTER_Y) - loadHalf,
                OdometryConfig.LOADING_ZONE_SIZE, OdometryConfig.LOADING_ZONE_SIZE);
        c.strokeRect(toDashX(OdometryConfig.LOADING_ZONE_BLUE_CENTER_X) - loadHalf, toDashY(OdometryConfig.LOADING_ZONE_BLUE_CENTER_Y) - loadHalf,
                OdometryConfig.LOADING_ZONE_SIZE, OdometryConfig.LOADING_ZONE_SIZE);

        // Gates
        c.setStroke(COLOR_RED);
        c.strokeLine(toDashX(OdometryConfig.GATE_RED_X) - 4, toDashY(OdometryConfig.GATE_RED_Y), toDashX(OdometryConfig.GATE_RED_X) + 4, toDashY(OdometryConfig.GATE_RED_Y));
        c.setStroke(COLOR_BLUE);
        c.strokeLine(toDashX(OdometryConfig.GATE_BLUE_X) - 4, toDashY(OdometryConfig.GATE_BLUE_Y), toDashX(OdometryConfig.GATE_BLUE_X) + 4, toDashY(OdometryConfig.GATE_BLUE_Y));

        // Gate zones
        double gateHalfW = OdometryConfig.GATE_ZONE_WIDTH / 2.0;
        double gateHalfD = OdometryConfig.GATE_ZONE_DEPTH / 2.0;
        c.setFill("rgba(255,0,0,0.2)");
        c.fillRect(toDashX(OdometryConfig.GATE_ZONE_RED_CENTER_X) - gateHalfW, toDashY(OdometryConfig.GATE_ZONE_RED_CENTER_Y) - gateHalfD,
                OdometryConfig.GATE_ZONE_WIDTH, OdometryConfig.GATE_ZONE_DEPTH);
        c.setFill("rgba(0,0,255,0.2)");
        c.fillRect(toDashX(OdometryConfig.GATE_ZONE_BLUE_CENTER_X) - gateHalfW, toDashY(OdometryConfig.GATE_ZONE_BLUE_CENTER_Y) - gateHalfD,
                OdometryConfig.GATE_ZONE_WIDTH, OdometryConfig.GATE_ZONE_DEPTH);

        // Secret tunnels
        c.setFill("rgba(255,0,0,0.35)");
        c.fillRect(toDashX(OdometryConfig.SECRET_TUNNEL_RED_X) - 1, toDashY(OdometryConfig.SECRET_TUNNEL_RED_Y1),
                2, OdometryConfig.SECRET_TUNNEL_RED_Y2 - OdometryConfig.SECRET_TUNNEL_RED_Y1);
        c.setFill("rgba(0,0,255,0.35)");
        c.fillRect(toDashX(OdometryConfig.SECRET_TUNNEL_BLUE_X) - 1, toDashY(OdometryConfig.SECRET_TUNNEL_BLUE_Y1),
                2, OdometryConfig.SECRET_TUNNEL_BLUE_Y2 - OdometryConfig.SECRET_TUNNEL_BLUE_Y1);

        // Artifact rows (PGP ordering left→right when looking toward targets)
        drawArtifactRows(c, motif);
    }

    private static void drawArtifactRows(Canvas c, ObeliskSignal.Order motif) {
        String[] gpp = new String[]{"G", "P", "P"};
        String[] pgp = new String[]{"P", "G", "P"};
        String[] ppg = new String[]{"P", "P", "G"};

        drawArtifactRow(c, OdometryConfig.RED_ARTIFACT_ROW_START_X, OdometryConfig.GPP_ROW_Y, gpp);
        drawArtifactRow(c, OdometryConfig.RED_ARTIFACT_ROW_START_X, OdometryConfig.PGP_ROW_Y, pgp);
        drawArtifactRow(c, OdometryConfig.RED_ARTIFACT_ROW_START_X, OdometryConfig.PPG_ROW_Y, ppg);

        drawArtifactRow(c, OdometryConfig.BLUE_ARTIFACT_ROW_START_X, OdometryConfig.GPP_ROW_Y, gpp);
        drawArtifactRow(c, OdometryConfig.BLUE_ARTIFACT_ROW_START_X, OdometryConfig.PGP_ROW_Y, pgp);
        drawArtifactRow(c, OdometryConfig.BLUE_ARTIFACT_ROW_START_X, OdometryConfig.PPG_ROW_Y, ppg);

        if (motif != null) {
            double rowY = (motif == ObeliskSignal.Order.GPP) ? OdometryConfig.GPP_ROW_Y
                    : (motif == ObeliskSignal.Order.PPG) ? OdometryConfig.PPG_ROW_Y : OdometryConfig.PGP_ROW_Y;
            c.setStroke("#FFFFFF");
            c.strokeLine(toDashX(OdometryConfig.LEFT_FIELD_X), toDashY(rowY + OdometryConfig.ARTIFACT_RADIUS * 1.5),
                    toDashX(OdometryConfig.RIGHT_FIELD_X), toDashY(rowY + OdometryConfig.ARTIFACT_RADIUS * 1.5));
        }
    }

    private static void drawArtifactRow(Canvas c, double startX, double y, String[] order) {
        for (int i = 0; i < order.length; i++) {
            String color = order[i].equals("G") ? COLOR_GREEN : COLOR_PURPLE;
            c.setFill(color);
            c.fillCircle(toDashX(startX + i * OdometryConfig.ARTIFACT_SPACING_X), toDashY(y), OdometryConfig.ARTIFACT_RADIUS);
        }
    }

    private static void drawPose(Canvas c, FieldPose pose) {
        double px = toDashX(pose.x);
        double py = toDashY(pose.y);
        c.setFill("#333333");
        c.fillCircle(px, py, 3.0);
        double headingRad = Math.toRadians(pose.headingDeg);
        double hx = px + 8.0 * Math.sin(headingRad);
        double hy = py + 8.0 * Math.cos(headingRad);
        c.setStroke("#000000");
        c.strokeLine(px, py, hx, hy);
    }

    /** Overlay the numeric pose values (inches, degrees) onto the field view. */
    private static void drawPoseText(Canvas c, FieldPose pose) {
        c.setStroke("#000000");
        c.setFill("#000000");
        double textX = toDashX(OdometryConfig.LEFT_FIELD_X) + 4;
        double textY = toDashY(OdometryConfig.TARGET_WALL_Y) - 6;
        c.strokeText(String.format("Pose: %.1f, %.1f, %.1f", pose.x, pose.y, pose.headingDeg),
                textX, textY, "#000000", 12);
    }

    private static void drawTriangle(Canvas c, double x1, double y1, double x2, double y2, double x3, double y3) {
        double ax = toDashX(x1), ay = toDashY(y1);
        double bx = toDashX(x2), by = toDashY(y2);
        double cx = toDashX(x3), cy = toDashY(y3);
        c.strokeLine(ax, ay, bx, by);
        c.strokeLine(bx, by, cx, cy);
        c.strokeLine(cx, cy, ax, ay);
    }

    private static double toDashX(double fieldX) {
        return fieldX; // X axes already aligned (centered field frame)
    }

    private static double toDashY(double fieldY) {
        return fieldY; // Odometry now uses the dashboard-centered frame directly
    }
}
