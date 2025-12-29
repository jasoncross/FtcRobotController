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
 * CHANGES (2025-12-29): Corrected dashboard heading arrow rotation to match
 *                        the odometry CCW-positive convention (0° = +Y) and
 *                        added configurable dashboard-only orientation
 *                        transforms for field visualization.
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
        if (pose != null) drawPose(c, pose);
    }

    private static void drawStatic(Canvas c, ObeliskSignal.Order motif) {
        // Mat background
        c.setFill(COLOR_GRAY);
        fillRectField(c, OdometryConfig.LEFT_FIELD_X, OdometryConfig.HUMAN_WALL_Y,
                OdometryConfig.FIELD_WIDTH, OdometryConfig.FIELD_LENGTH);

        // Border
        c.setStroke(COLOR_WHITE);
        strokeRectField(c, OdometryConfig.LEFT_FIELD_X, OdometryConfig.HUMAN_WALL_Y,
                OdometryConfig.FIELD_WIDTH, OdometryConfig.FIELD_LENGTH);

        // Tile grid every 24"
        c.setStroke(COLOR_TILE);
        for (double x = OdometryConfig.LEFT_FIELD_X; x <= OdometryConfig.RIGHT_FIELD_X; x += 24.0) {
            strokeLineField(c, x, OdometryConfig.HUMAN_WALL_Y, x, OdometryConfig.TARGET_WALL_Y);
        }
        for (double y = OdometryConfig.HUMAN_WALL_Y; y <= OdometryConfig.TARGET_WALL_Y; y += 24.0) {
            strokeLineField(c, OdometryConfig.LEFT_FIELD_X, y, OdometryConfig.RIGHT_FIELD_X, y);
        }

        // Launch lines
        c.setStroke(COLOR_WHITE);
        strokeLineField(c, OdometryConfig.LEFT_FIELD_X, OdometryConfig.HUMAN_LAUNCH_LINE_Y,
                OdometryConfig.RIGHT_FIELD_X, OdometryConfig.HUMAN_LAUNCH_LINE_Y);
        strokeLineField(c, OdometryConfig.LEFT_FIELD_X, OdometryConfig.TARGET_LAUNCH_LINE_Y,
                OdometryConfig.RIGHT_FIELD_X, OdometryConfig.TARGET_LAUNCH_LINE_Y);

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
        fillRectField(c, OdometryConfig.GOAL_RED_X - 2, OdometryConfig.GOAL_RED_Y - 2, 4, 4);
        c.setFill(COLOR_BLUE);
        fillRectField(c, OdometryConfig.GOAL_BLUE_X - 2, OdometryConfig.GOAL_BLUE_Y - 2, 4, 4);

        // Classifiers
        c.setStroke(COLOR_RED);
        strokeRectField(c, OdometryConfig.CLASSIFIER_RED_X - 3, OdometryConfig.CLASSIFIER_RED_Y - 3, 6, 6);
        c.setStroke(COLOR_BLUE);
        strokeRectField(c, OdometryConfig.CLASSIFIER_BLUE_X - 3, OdometryConfig.CLASSIFIER_BLUE_Y - 3, 6, 6);

        // Base zones
        double baseHalf = OdometryConfig.BASE_ZONE_SIZE / 2.0;
        c.setStroke(COLOR_RED);
        strokeRectField(c, OdometryConfig.BASE_ZONE_RED_CENTER_X - baseHalf, OdometryConfig.BASE_ZONE_RED_CENTER_Y - baseHalf,
                OdometryConfig.BASE_ZONE_SIZE, OdometryConfig.BASE_ZONE_SIZE);
        c.setStroke(COLOR_BLUE);
        strokeRectField(c, OdometryConfig.BASE_ZONE_BLUE_CENTER_X - baseHalf, OdometryConfig.BASE_ZONE_BLUE_CENTER_Y - baseHalf,
                OdometryConfig.BASE_ZONE_SIZE, OdometryConfig.BASE_ZONE_SIZE);

        // Loading zones
        double loadHalf = OdometryConfig.LOADING_ZONE_SIZE / 2.0;
        c.setStroke(COLOR_WHITE);
        strokeRectField(c, OdometryConfig.LOADING_ZONE_RED_CENTER_X - loadHalf, OdometryConfig.LOADING_ZONE_RED_CENTER_Y - loadHalf,
                OdometryConfig.LOADING_ZONE_SIZE, OdometryConfig.LOADING_ZONE_SIZE);
        strokeRectField(c, OdometryConfig.LOADING_ZONE_BLUE_CENTER_X - loadHalf, OdometryConfig.LOADING_ZONE_BLUE_CENTER_Y - loadHalf,
                OdometryConfig.LOADING_ZONE_SIZE, OdometryConfig.LOADING_ZONE_SIZE);

        // Gates
        c.setStroke(COLOR_RED);
        strokeLineField(c, OdometryConfig.GATE_RED_X - 4, OdometryConfig.GATE_RED_Y,
                OdometryConfig.GATE_RED_X + 4, OdometryConfig.GATE_RED_Y);
        c.setStroke(COLOR_BLUE);
        strokeLineField(c, OdometryConfig.GATE_BLUE_X - 4, OdometryConfig.GATE_BLUE_Y,
                OdometryConfig.GATE_BLUE_X + 4, OdometryConfig.GATE_BLUE_Y);

        // Gate zones
        double gateHalfW = OdometryConfig.GATE_ZONE_WIDTH / 2.0;
        double gateHalfD = OdometryConfig.GATE_ZONE_DEPTH / 2.0;
        c.setFill("rgba(255,0,0,0.2)");
        fillRectField(c, OdometryConfig.GATE_ZONE_RED_CENTER_X - gateHalfW, OdometryConfig.GATE_ZONE_RED_CENTER_Y - gateHalfD,
                OdometryConfig.GATE_ZONE_WIDTH, OdometryConfig.GATE_ZONE_DEPTH);
        c.setFill("rgba(0,0,255,0.2)");
        fillRectField(c, OdometryConfig.GATE_ZONE_BLUE_CENTER_X - gateHalfW, OdometryConfig.GATE_ZONE_BLUE_CENTER_Y - gateHalfD,
                OdometryConfig.GATE_ZONE_WIDTH, OdometryConfig.GATE_ZONE_DEPTH);

        // Secret tunnels
        c.setFill("rgba(255,0,0,0.35)");
        fillRectField(c, OdometryConfig.SECRET_TUNNEL_RED_X - 1, OdometryConfig.SECRET_TUNNEL_RED_Y1,
                2, OdometryConfig.SECRET_TUNNEL_RED_Y2 - OdometryConfig.SECRET_TUNNEL_RED_Y1);
        c.setFill("rgba(0,0,255,0.35)");
        fillRectField(c, OdometryConfig.SECRET_TUNNEL_BLUE_X - 1, OdometryConfig.SECRET_TUNNEL_BLUE_Y1,
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
            strokeLineField(c, OdometryConfig.LEFT_FIELD_X, rowY + OdometryConfig.ARTIFACT_RADIUS * 1.5,
                    OdometryConfig.RIGHT_FIELD_X, rowY + OdometryConfig.ARTIFACT_RADIUS * 1.5);
        }
    }

    private static void drawArtifactRow(Canvas c, double startX, double y, String[] order) {
        for (int i = 0; i < order.length; i++) {
            String color = order[i].equals("G") ? COLOR_GREEN : COLOR_PURPLE;
            c.setFill(color);
            fillCircleField(c, startX + i * OdometryConfig.ARTIFACT_SPACING_X, y, OdometryConfig.ARTIFACT_RADIUS);
        }
    }

    private static void drawPose(Canvas c, FieldPose pose) {
        DashboardPoint center = toDashboardPoint(pose.x, pose.y);
        c.setFill("#333333");
        c.fillCircle(center.x, center.y, 3.0);

        double headingRad = Math.toRadians(-pose.headingDeg); // draw CCW-positive heading in dashboard's CW-positive frame
        double fieldHx = pose.x + 8.0 * Math.sin(headingRad);
        double fieldHy = pose.y + 8.0 * Math.cos(headingRad);
        DashboardPoint head = toDashboardPoint(fieldHx, fieldHy);

        c.setStroke("#000000");
        c.strokeLine(center.x, center.y, head.x, head.y);
    }

    private static void drawTriangle(Canvas c, double x1, double y1, double x2, double y2, double x3, double y3) {
        strokeLineField(c, x1, y1, x2, y2);
        strokeLineField(c, x2, y2, x3, y3);
        strokeLineField(c, x3, y3, x1, y1);
    }

    /*
     * DASHBOARD ORIENTATION CHECK (manual verification):
     * 1) Place the robot at a known field corner or (0,0) origin and verify the dot
     *    appears in the same corner/center on the dashboard overlay.
     * 2) Drive toward the targets; confirm the pose dot moves upward on the field view.
     * 3) Confirm the static elements (targets, human player zones) match expected
     *    left/right/top/bottom after applying the dashboard transform.
     */
    private static DashboardPoint toDashboardPoint(double fieldX, double fieldY) {
        double x = fieldX;
        double y = fieldY;
        if (OdometryConfig.DASHBOARD_MIRROR_X) {
            x = -x;
        }
        if (OdometryConfig.DASHBOARD_MIRROR_Y) {
            y = -y;
        }

        double rotationDeg = normalizeRotationDeg(OdometryConfig.DASHBOARD_ROTATION_DEG);
        double rad = Math.toRadians(rotationDeg);
        double cos = Math.cos(rad);
        double sin = Math.sin(rad);
        double rx = x * cos - y * sin;
        double ry = x * sin + y * cos;
        return new DashboardPoint(rx, ry);
    }

    private static double normalizeRotationDeg(double degrees) {
        double normalized = degrees % 360.0;
        if (normalized < 0) {
            normalized += 360.0;
        }
        return normalized;
    }

    private static void strokeLineField(Canvas c, double x1, double y1, double x2, double y2) {
        DashboardPoint a = toDashboardPoint(x1, y1);
        DashboardPoint b = toDashboardPoint(x2, y2);
        c.strokeLine(a.x, a.y, b.x, b.y);
    }

    private static void fillCircleField(Canvas c, double x, double y, double radius) {
        DashboardPoint center = toDashboardPoint(x, y);
        c.fillCircle(center.x, center.y, radius);
    }

    private static void strokeRectField(Canvas c, double x, double y, double width, double height) {
        DashboardRect rect = toDashboardRect(x, y, width, height);
        c.strokeRect(rect.x, rect.y, rect.width, rect.height);
    }

    private static void fillRectField(Canvas c, double x, double y, double width, double height) {
        DashboardRect rect = toDashboardRect(x, y, width, height);
        c.fillRect(rect.x, rect.y, rect.width, rect.height);
    }

    private static DashboardRect toDashboardRect(double x, double y, double width, double height) {
        DashboardPoint p1 = toDashboardPoint(x, y);
        DashboardPoint p2 = toDashboardPoint(x + width, y);
        DashboardPoint p3 = toDashboardPoint(x + width, y + height);
        DashboardPoint p4 = toDashboardPoint(x, y + height);

        double minX = Math.min(Math.min(p1.x, p2.x), Math.min(p3.x, p4.x));
        double maxX = Math.max(Math.max(p1.x, p2.x), Math.max(p3.x, p4.x));
        double minY = Math.min(Math.min(p1.y, p2.y), Math.min(p3.y, p4.y));
        double maxY = Math.max(Math.max(p1.y, p2.y), Math.max(p3.y, p4.y));
        return new DashboardRect(minX, minY, maxX - minX, maxY - minY);
    }

    private static final class DashboardPoint {
        private final double x;
        private final double y;

        private DashboardPoint(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    private static final class DashboardRect {
        private final double x;
        private final double y;
        private final double width;
        private final double height;

        private DashboardRect(double x, double y, double width, double height) {
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
        }
    }
}
