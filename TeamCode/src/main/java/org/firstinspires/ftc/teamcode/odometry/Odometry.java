package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LimelightHelpers;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.OdometryConfig;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.drive.Drivebase;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import static java.lang.Math.*;

/*
 * FILE: Odometry.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/odometry/
 *
 * PURPOSE
 *   - Fuse drivetrain encoder deltas, IMU heading, and optional Limelight XY
 *     corrections into a stable field pose anchored at the robot center.
 *   - Publish a consistent coordinate frame for TeleOp, Auto, and dashboard
 *     drawing utilities without duplicating geometry math across callers.
 *
 * NOTES
 *   - Heading comes from Drivebase.heading() plus the configurable IMU offset.
 *   - Limelight corrections blend position only (X/Y); yaw is always IMU-only.
 *
 * CHANGES (2025-11-26): Corrected odometry sign conventions so +X is right and
 *                        +Y is toward the targets, normalized IMU heading to
 *                        the shared frame (0° facing +Y, +CCW), and reused the
 *                        same heading basis for vision pose fusion.
 * CHANGES (2025-11-25): Added goal-tag pose fusion that leverages config tag poses,
 *                        camera offsets, and pitch/yaw to compute the robot center
 *                        from detections, enabling Auto→TeleOp pose carryover.
 * CHANGES (2025-12-11): Switched to the FTC field-center frame (+Y toward targets),
 *                        removed webcam-based fusion, and added Limelight-only
 *                        XY blending with reacquire-friendly gating and bounded
 *                        correction steps.
 * CHANGES (2025-12-12): Updated Limelight pose accessors for current API field
 *                        members to restore build compatibility on center-frame
 *                        odometry.
 * CHANGES (2025-12-29): Added Limelight localization filters, MT2 yaw feeding,
 *                        and field-bounds gating so vision fusion stays stable
 *                        and goal-tag-only for pose updates.
 * CHANGES (2025-12-29): Added per-loop vision debug telemetry and rejection
 *                        reasons to diagnose Limelight fusion decisions.
 * CHANGES (2025-12-29): Removed NetworkTables localization filter fallback so
 *                        builds without WPILib NTCore remain compatible.
 * CHANGES (2025-12-29): Switched MT2 yaw feeding + localization filtering to
 *                        LimelightHelpers so FTC builds avoid WPILib NTCore.
 * CHANGES (2025-12-29): Added an odometry-only distance scale to isolate Auto
 *                        move distance from pose calibration.
 * CHANGES (2025-12-30): Routed MT2 yaw feed through LimelightHelpers static
 *                        orientation calls (SetRobotOrientation / setRobotOrientation),
 *                        added yaw/filter age telemetry, and preferred Helpers
 *                        fiducial override calls with Limelight3A fallback.
 * CHANGES (2025-12-30): Rejected Limelight fusion updates when obelisk tags
 *                        are visible (with debug override) and expanded the
 *                        VisionDbg line to surface tag participation details.
 * CHANGES (2025-12-30): Added missing List import for fiducial ID collection
 *                        helpers to keep Odometry builds clean.
 */
public class Odometry {

    private final Drivebase drive;
    private final Limelight3A limelight;

    private final double[] lastWheelInches = new double[4];
    private final int[] lastWheelTicks = new int[4];
    private boolean initialized = false;
    private FieldPose pose = new FieldPose();

    private long lastUpdateNanos = -1L;
    private long lastAcceptedVisionMs = -1L;
    private int validVisionStreak = 0;
    private FieldPose lastVisionPose = null;
    private FieldPose lastRawVisionPose = null;
    private FieldPose lastRawBluePose = null;
    private FieldPose lastRawRedPose = null;
    private Double lastMt2Xmeters = null;
    private Double lastMt2Ymeters = null;
    private Double lastMt2Xin = null;
    private Double lastMt2Yin = null;
    private String lastPoseSrc = "--";
    private boolean lastMt2Expected = false;
    private boolean lastMt2Active = false;
    private int lastMt2TagCount = -1;
    private Double lastBoundsTestX = null;
    private Double lastBoundsTestY = null;
    private Double lastBoundsMax = null;
    private final double[] lastWheelDeltaRaw = new double[4];
    private final double[] lastWheelDeltaScaled = new double[4];
    private final int[] lastWheelDeltaTicks = new int[4];
    private double lastRawErrorMag = Double.NaN;
    private boolean lastYawFeedOk = false;
    private Double lastYawSentDeg = null;
    private Long lastVisionAgeMs = null;
    private Integer lastPrimaryTagId = null;
    private int lastVisibleTagCount = 0;
    private String lastRejectReason = "OK";
    private String visionDebugLine = null;
    private String odometryDebugLine = null;
    private long lastLocalizationFilterMs = 0L;
    private boolean lastLocalizationFilterOk = false;
    private int lastLocalizationFilterHash = 0;
    private long lastYawFeedMs = 0L;
    private int[] lastLocalizationIds = null;
    private boolean lastLocalizationTidOk = false;
    private int[] lastVisibleIds = null;
    private int[] lastObeliskIds = null;
    private boolean lastObeliskSeen = false;

    private static final double M_TO_IN = 39.37007874;

    public Odometry(Drivebase drive, Limelight3A limelight) {
        this.drive = drive;
        this.limelight = limelight;
        LimelightHelpers.registerLimelight(VisionConfig.LimelightFusion.LL_NT_NAME, limelight);
    }

    /** Initialize odometry to a known pose (robot center). */
    public void setPose(double x, double y, double headingDeg) {
        pose.x = x;
        pose.y = y;
        pose.headingDeg = headingDeg;
        double[] cur = drive.getWheelPositionsInches();
        int[] ticks = drive.getWheelPositionsTicks();
        System.arraycopy(cur, 0, lastWheelInches, 0, lastWheelInches.length);
        System.arraycopy(ticks, 0, lastWheelTicks, 0, lastWheelTicks.length);
        initialized = true;
    }

    public FieldPose getPose() { return pose.copy(); }

    /** Returns the last accepted Limelight XY pose (field frame) if available. */
    public FieldPose getLastVisionPose() { return lastVisionPose == null ? null : lastVisionPose.copy(); }

    /** Returns the latest Limelight fusion debug line (null when unavailable). */
    public String getVisionDebugLine() { return visionDebugLine; }

    /** Returns the latest odometry debug line (null when unavailable). */
    public String getOdometryDebugLine() { return odometryDebugLine; }

    /**
     * Update pose using wheel deltas + IMU heading. Optionally blend Limelight
     * XY corrections (heading remains IMU-only).
     */
    public FieldPose update() {
        double heading = normalizeHeading(drive.heading());

        int[] ticks = drive.getWheelPositionsTicks();
        double[] wheels = drive.getWheelPositionsInches();
        long nowNs = System.nanoTime();
        double dtSec = (lastUpdateNanos > 0) ? (nowNs - lastUpdateNanos) / 1e9 : 0.0;
        lastUpdateNanos = nowNs;

        if (!initialized) {
            System.arraycopy(wheels, 0, lastWheelInches, 0, wheels.length);
            System.arraycopy(ticks, 0, lastWheelTicks, 0, ticks.length);
            pose.headingDeg = heading;
            initialized = true;
            return pose.copy();
        }

        lastWheelDeltaTicks[0] = ticks[0] - lastWheelTicks[0];
        lastWheelDeltaTicks[1] = ticks[1] - lastWheelTicks[1];
        lastWheelDeltaTicks[2] = ticks[2] - lastWheelTicks[2];
        lastWheelDeltaTicks[3] = ticks[3] - lastWheelTicks[3];
        System.arraycopy(ticks, 0, lastWheelTicks, 0, ticks.length);

        double fl = wheels[0] - lastWheelInches[0];
        double fr = wheels[1] - lastWheelInches[1];
        double bl = wheels[2] - lastWheelInches[2];
        double br = wheels[3] - lastWheelInches[3];
        System.arraycopy(wheels, 0, lastWheelInches, 0, wheels.length);

        lastWheelDeltaRaw[0] = fl;
        lastWheelDeltaRaw[1] = fr;
        lastWheelDeltaRaw[2] = bl;
        lastWheelDeltaRaw[3] = br;

        double scale = OdometryConfig.ODOMETRY_DISTANCE_SCALE;
        double flScaled = fl * scale;
        double frScaled = fr * scale;
        double blScaled = bl * scale;
        double brScaled = br * scale;

        lastWheelDeltaScaled[0] = flScaled;
        lastWheelDeltaScaled[1] = frScaled;
        lastWheelDeltaScaled[2] = blScaled;
        lastWheelDeltaScaled[3] = brScaled;

        double forward = -(flScaled + frScaled + blScaled + brScaled) / 4.0; // +Y toward targets (invert to match field frame)
        double strafeRaw = -(flScaled - frScaled - blScaled + brScaled) / 4.0; // +X right (invert to match field frame)
        double strafe = strafeRaw / Drivebase.STRAFE_CORRECTION;

        double hRad = toRadians(heading);
        double dx = strafe * cos(hRad) - forward * sin(hRad);
        double dy = strafe * sin(hRad) + forward * cos(hRad);

        double speedInPerS = (dtSec > 1e-6) ? hypot(dx, dy) / dtSec : 0.0;
        double turnRateDegPerS = (dtSec > 1e-6) ? abs(normHeading(heading - pose.headingDeg)) / dtSec : 0.0;

        FieldPose integrated = new FieldPose(
                pose.x + dx,
                pose.y + dy,
                heading
        );

        FieldPose blended = applyLimelightFusion(integrated, speedInPerS, turnRateDegPerS);
        pose = lowPass(pose, blended, OdometryConfig.POSE_FILTER_STRENGTH);
        updateVisionDebugLine(heading);
        updateOdometryDebugLine();
        return pose.copy();
    }

    private FieldPose applyLimelightFusion(FieldPose base, double speedInPerS, double turnRateDegPerS) {
        resetVisionDebug();
        if (!VisionConfig.LimelightFusion.ENABLE_POSE_FUSION || limelight == null) {
            validVisionStreak = 0;
            lastRejectReason = "OFF";
            return base;
        }

        applyLimelightLocalizationFilter();
        feedLimelightYaw(base.headingDeg);

        if (speedInPerS > VisionConfig.LimelightFusion.MAX_SPEED_IN_PER_S) {
            lastRejectReason = "SPEED";
            return base;
        }
        if (turnRateDegPerS > VisionConfig.LimelightFusion.MAX_TURN_RATE_DEG_PER_S) {
            lastRejectReason = "TURN";
            return base;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            validVisionStreak = 0;
            lastRejectReason = "NULL";
            return base;
        }
        lastPrimaryTagId = readPrimaryTagId(result);
        lastVisibleTagCount = countFiducials(result);
        lastVisibleIds = readVisibleFiducialIds(result);
        lastObeliskIds = filterObeliskIds(lastVisibleIds);
        lastObeliskSeen = lastObeliskIds.length > 0;
        lastLocalizationTidOk = isTagAllowedForLocalization(lastPrimaryTagId, lastLocalizationIds);

        if (lastObeliskSeen && VisionConfig.LimelightFusion.DEBUG_REJECT_ON_OBELISK) {
            validVisionStreak = 0;
            lastRejectReason = "OBELISK";
            return base;
        }

        Pose3D pose3D = selectPose(result);
        lastMt2Expected = VisionConfig.LimelightFusion.PREFER_MEGA_TAG_2;
        lastMt2TagCount = lastVisibleTagCount;
        long yawAgeMs = lastYawFeedOk ? Math.max(0L, System.currentTimeMillis() - lastYawFeedMs) : Long.MAX_VALUE;
        lastMt2Active = lastMt2Expected
                && lastYawFeedOk
                && yawAgeMs <= VisionConfig.LimelightFusion.YAW_MAX_AGE_MS
                && lastVisibleTagCount > 0;

        if (pose3D == null || pose3D.getPosition() == null) {
            validVisionStreak = 0;
            lastRejectReason = "NULL";
            return base;
        }

        long nowMs = System.currentTimeMillis();
        Long timestampMs = readTimestampMs(result);
        if (timestampMs != null) {
            long ageMs = nowMs - timestampMs;
            lastVisionAgeMs = ageMs;
            if (ageMs > VisionConfig.LimelightFusion.MAX_AGE_MS) {
                validVisionStreak = 0;
                lastRejectReason = "AGE";
                return base;
            }
        }

        validVisionStreak++;
        if (validVisionStreak < VisionConfig.LimelightFusion.MIN_VALID_FRAMES) {
            lastRejectReason = "STREAK";
            return base;
        }

        updateRawFrameDebug(result);

        LlXySample sample = transformLimelightFieldToTeamField(
                pose3D.getPosition().x,
                pose3D.getPosition().y,
                true);
        updateRawMt2Debug(sample);
        double vx = sample.fx;
        double vy = sample.fy;
        lastRawVisionPose = new FieldPose(sample.tx, sample.ty, base.headingDeg);
        double maxAllowed = boundsMaxIn();
        lastBoundsTestX = vx;
        lastBoundsTestY = vy;
        lastBoundsMax = maxAllowed;
        if (!isWithinFieldBounds(vx, vy, maxAllowed)) {
            validVisionStreak = 0;
            lastRejectReason = "BOUNDS";
            return base;
        }


        long sinceLast = (lastAcceptedVisionMs < 0) ? Long.MAX_VALUE : nowMs - lastAcceptedVisionMs;
        boolean reacquire = sinceLast > VisionConfig.LimelightFusion.REACQUIRE_AFTER_MS;
        double maxJump = reacquire
                ? VisionConfig.LimelightFusion.MAX_POS_JUMP_IN_REACQUIRE
                : VisionConfig.LimelightFusion.MAX_POS_JUMP_IN_NORMAL;
        double alpha = reacquire
                ? VisionConfig.LimelightFusion.FUSION_ALPHA_REACQUIRE
                : VisionConfig.LimelightFusion.FUSION_ALPHA_NORMAL;

        double ex = vx - base.x;
        double ey = vy - base.y;
        double errorMag = hypot(ex, ey);
        lastRawErrorMag = errorMag;
        if (errorMag > maxJump) {
            lastRejectReason = "JUMP";
            return base;
        }

        double[] step = clampMagnitude(ex, ey, VisionConfig.LimelightFusion.MAX_CORRECTION_STEP_IN);
        FieldPose corrected = new FieldPose(
                base.x + step[0] * alpha,
                base.y + step[1] * alpha,
                base.headingDeg
        );
        lastBoundsTestX = corrected.x;
        lastBoundsTestY = corrected.y;
        lastBoundsMax = maxAllowed;
        if (!isWithinFieldBounds(corrected.x, corrected.y, maxAllowed)) {
            validVisionStreak = 0;
            lastRejectReason = "BOUNDS";
            return base;
        }
        lastVisionPose = new FieldPose(vx, vy, base.headingDeg);
        lastAcceptedVisionMs = nowMs;
        lastRejectReason = "OK";
        return corrected;
    }

    private Pose3D selectPose(LLResult result) {
        if (result == null) return null;
        Pose3D pose = null;
        if (VisionConfig.LimelightFusion.USE_LLRESULT_BOTPOSE_MT2) {
            pose = readPoseFromResult(result, "getBotpose_MT2");
            if (pose != null) {
                lastPoseSrc = "LLResult.getBotpose_MT2";
                return pose;
            }
        }
        pose = result.getBotpose();
        lastPoseSrc = "LLResult.getBotpose";
        return pose;
    }

    private Pose3D readPoseFromResult(LLResult result, String method) {
        try {
            Object value = result.getClass().getMethod(method).invoke(result);
            return (value instanceof Pose3D) ? (Pose3D) value : null;
        } catch (Throwable ignored) {
        }
        return null;
    }

    private LlXySample transformLimelightFieldToTeamField(double xMeters, double yMeters, boolean applyOffset) {
        LlXySample sample = new LlXySample();
        sample.xMeters = xMeters;
        sample.yMeters = yMeters;
        sample.xIn = xMeters * M_TO_IN;
        sample.yIn = yMeters * M_TO_IN;

        sample.tx = VisionConfig.LimelightFusion.AXIS_SWAP_XY ? sample.yIn : sample.xIn;
        sample.ty = VisionConfig.LimelightFusion.AXIS_SWAP_XY ? sample.xIn : sample.yIn;

        sample.tx *= VisionConfig.LimelightFusion.X_SIGN;
        sample.ty *= VisionConfig.LimelightFusion.Y_SIGN;

        if (VisionConfig.LimelightFusion.APPLY_CENTER_SHIFT) {
            sample.tx -= VisionConfig.LimelightFusion.FIELD_HALF_IN;
            sample.ty -= VisionConfig.LimelightFusion.FIELD_HALF_IN;
        }

        sample.fx = sample.tx;
        sample.fy = sample.ty;
        if (applyOffset) {
            sample.fx += VisionConfig.LimelightFusion.X_OFFSET_IN;
            sample.fy += VisionConfig.LimelightFusion.Y_OFFSET_IN;
        }
        return sample;
    }

    private void applyLimelightLocalizationFilter() {
        if (!VisionConfig.LimelightFusion.ENABLE_LOCALIZATION_TAG_FILTER) {
            lastLocalizationFilterOk = false;
            lastLocalizationIds = null;
            return;
        }

        int[] base = VisionConfig.LimelightFusion.LOCALIZATION_VALID_TAG_IDS;
        if (base == null || base.length == 0) {
            base = new int[]{VisionConfig.GOAL_TAG_BLUE, VisionConfig.GOAL_TAG_RED};
        }
        int[] excluded = VisionConfig.LimelightFusion.LOCALIZATION_EXCLUDED_TAG_IDS;
        int[] ids = filterIds(base, excluded);
        if (ids.length == 0) {
            lastLocalizationFilterOk = false;
            lastLocalizationIds = null;
            return;
        }

        long now = System.currentTimeMillis();
        int hash = Arrays.hashCode(ids);
        boolean shouldApply = VisionConfig.LimelightFusion.LOCALIZATION_FILTER_APPLY_EVERY_FRAME
                || (now - lastLocalizationFilterMs) >= 500L
                || hash != lastLocalizationFilterHash;
        if (!shouldApply) return;

        boolean ok = invokeHelpersFiducialOverride(ids);
        if (!ok) {
            ok = invokeLimelightFiducialFilter(ids);
        }

        lastLocalizationFilterOk = ok;
        lastLocalizationIds = ids;
        if (ok) {
            lastLocalizationFilterMs = now;
            lastLocalizationFilterHash = hash;
        }
    }


    private void feedLimelightYaw(double headingDeg) {
        lastYawFeedOk = false;
        lastYawSentDeg = null;
        if (!VisionConfig.LimelightFusion.PREFER_MEGA_TAG_2) return;
        lastYawFeedOk = invokeHelpersYawFeed(headingDeg);
    }

    private boolean invokeHelpersYawFeed(double headingDeg) {
        try {
            return invokeLimelightHelpersOrientation("SetRobotOrientation", headingDeg)
                    || invokeLimelightHelpersOrientation("setRobotOrientation", headingDeg);
        } catch (Throwable ignored) {
        }
        return false;
    }

    private boolean invokeLimelightHelpersOrientation(String methodName, double headingDeg) {
        try {
            LimelightHelpers.class.getMethod(
                    methodName,
                    String.class,
                    double.class,
                    double.class,
                    double.class,
                    double.class,
                    double.class,
                    double.class
            ).invoke(
                    null,
                    VisionConfig.LimelightFusion.LL_NT_NAME,
                    headingDeg,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
            );
            lastYawSentDeg = headingDeg;
            lastYawFeedMs = System.currentTimeMillis();
            return true;
        } catch (Throwable ignored) {
        }
        return false;
    }

    private boolean invokeHelpersFiducialOverride(int[] ids) {
        try {
            return invokeLimelightHelpersFiducialOverride("SetFiducialIDFiltersOverride", ids)
                    || invokeLimelightHelpersFiducialOverride("setFiducialIDFiltersOverride", ids);
        } catch (Throwable ignored) {
        }
        return false;
    }

    private boolean invokeLimelightHelpersFiducialOverride(String methodName, int[] ids) {
        try {
            LimelightHelpers.class.getMethod(methodName, String.class, int[].class)
                    .invoke(null, VisionConfig.LimelightFusion.LL_NT_NAME, (Object) ids);
            return true;
        } catch (Throwable ignored) {
        }
        return false;
    }

    private boolean invokeLimelightFiducialFilter(int[] ids) {
        if (limelight == null) return false;
        try {
            limelight.getClass().getMethod("setFiducialIDFilters", int[].class)
                    .invoke(limelight, (Object) ids);
            return true;
        } catch (Throwable ignored) {
        }
        return false;
    }

    private void resetVisionDebug() {
        lastRawVisionPose = null;
        lastRawBluePose = null;
        lastRawRedPose = null;
        lastMt2Xmeters = null;
        lastMt2Ymeters = null;
        lastMt2Xin = null;
        lastMt2Yin = null;
        lastPoseSrc = "--";
        lastMt2Expected = false;
        lastMt2Active = false;
        lastMt2TagCount = -1;
        lastBoundsTestX = null;
        lastBoundsTestY = null;
        lastBoundsMax = null;
        lastLocalizationIds = null;
        lastLocalizationTidOk = false;
        lastVisibleIds = null;
        lastObeliskIds = null;
        lastObeliskSeen = false;
        lastRawErrorMag = Double.NaN;
        lastVisionAgeMs = null;
        lastPrimaryTagId = null;
        lastVisibleTagCount = 0;
        lastRejectReason = "OK";
    }

    private void updateVisionDebugLine(double headingDeg) {
        String rawStr = formatPoint(lastRawVisionPose);
        String rawBlueStr = formatPoint(lastRawBluePose);
        String rawRedStr = formatPoint(lastRawRedPose);
        String llXYmStr = formatXYMeters();
        String llXYinStr = formatXYInches();
        String testStr = formatPair(lastBoundsTestX, lastBoundsTestY);
        String locIdsStr = formatIds(lastLocalizationIds);
        String visibleIdsStr = formatIds(lastVisibleIds);
        String obeliskIdsStr = formatIds(lastObeliskIds);
        String tidOkStr = String.valueOf(lastLocalizationTidOk);
        String accStr = formatPoint(lastVisionPose);
        String fusedStr = formatPoint(pose);
        String ageStr = (lastVisionAgeMs == null) ? "--" : String.format(Locale.US, "%d", lastVisionAgeMs);
        String tidStr = (lastPrimaryTagId == null) ? "--" : String.valueOf(lastPrimaryTagId);
        String errStr = Double.isFinite(lastRawErrorMag) ? String.format(Locale.US, "%.1f", lastRawErrorMag) : "--";
        String yawSentStr = (lastYawSentDeg == null) ? "--" : String.format(Locale.US, "%.1f", lastYawSentDeg);
        String yawAgeStr = (lastYawFeedOk && lastYawFeedMs > 0L)
                ? String.format(Locale.US, "%d", Math.max(0L, System.currentTimeMillis() - lastYawFeedMs))
                : "--";
        String fltAgeStr = (lastLocalizationFilterMs > 0L)
                ? String.format(Locale.US, "%d", Math.max(0L, System.currentTimeMillis() - lastLocalizationFilterMs))
                : "--";
        String boundStr = (lastBoundsMax == null) ? "--" : String.format(Locale.US, "%.0f", lastBoundsMax);
        boolean accepted = "OK".equals(lastRejectReason);
        String baseLine = String.format(Locale.US,
                "VisionDbg h=%.1f yawOk=%s yawSent=%s yawAgeMs=%s fltOk=%s fltAgeMs=%s flt=%s locIDs=%s tidOk=%s ll=%s src=%s mt2Exp=%s mt2Act=%s tags=%d age=%s tid=%s n=%d visibleIDs=%s obeliskSeen=%s obeliskIDs=%s llm=%s lli=%s raw=%s acc=%s fused=%s err=%s accepted=%s rejectReason=%s bnd=%s test=%s",
                headingDeg,
                lastYawFeedOk,
                yawSentStr,
                yawAgeStr,
                lastLocalizationFilterOk,
                fltAgeStr,
                VisionConfig.LimelightFusion.ENABLE_LOCALIZATION_TAG_FILTER,
                locIdsStr,
                tidOkStr,
                VisionConfig.LimelightFusion.LL_NT_NAME,
                lastPoseSrc,
                lastMt2Expected,
                lastMt2Active,
                lastMt2TagCount,
                ageStr,
                tidStr,
                lastVisibleTagCount,
                visibleIdsStr,
                lastObeliskSeen,
                obeliskIdsStr,
                llXYmStr,
                llXYinStr,
                rawStr,
                accStr,
                fusedStr,
                errStr,
                accepted,
                lastRejectReason,
                boundStr,
                testStr);
        if (VisionConfig.LimelightFusion.DEBUG_VERBOSE_VISION) {
            visionDebugLine = baseLine + String.format(Locale.US, " rawBlue=%s rawRed=%s", rawBlueStr, rawRedStr);
        } else {
            visionDebugLine = baseLine;
        }
    }

    private String formatXYMeters() {
        if (lastMt2Xmeters == null || lastMt2Ymeters == null) return "--,--";
        return String.format(Locale.US, "%.1f,%.1f", lastMt2Xmeters, lastMt2Ymeters);
    }

    private String formatXYInches() {
        if (lastMt2Xin == null || lastMt2Yin == null) return "--,--";
        return String.format(Locale.US, "%.1f,%.1f", lastMt2Xin, lastMt2Yin);
    }

    private void updateRawFrameDebug(LLResult result) {
        if (!VisionConfig.LimelightFusion.DEBUG_VERBOSE_VISION) return;
        Pose3D bluePose = readMt2PoseFromHelpers(result,
                "getBotposeMT2Blue",
                "getBotpose_MT2_WPIBlue",
                "getBotpose_MT2_Blue",
                "getBotpose_MT2_blue");
        Pose3D redPose = readMt2PoseFromHelpers(result,
                "getBotposeMT2Red",
                "getBotpose_MT2_WPIRed",
                "getBotpose_MT2_Red",
                "getBotpose_MT2_red");

        if (bluePose != null && bluePose.getPosition() != null) {
            LlXySample blueSample = transformLimelightFieldToTeamField(
                    bluePose.getPosition().x,
                    bluePose.getPosition().y,
                    false);
            lastRawBluePose = new FieldPose(blueSample.tx, blueSample.ty, pose.headingDeg);
        }

        if (redPose != null && redPose.getPosition() != null) {
            LlXySample redSample = transformLimelightFieldToTeamField(
                    redPose.getPosition().x,
                    redPose.getPosition().y,
                    false);
            lastRawRedPose = new FieldPose(redSample.tx, redSample.ty, pose.headingDeg);
        }
    }

    private Pose3D readMt2PoseFromHelpers(LLResult result, String helperMethod, String... resultMethods) {
        if (result == null) return null;
        try {
            Object value = LimelightHelpers.class
                    .getMethod(helperMethod, LLResult.class)
                    .invoke(null, result);
            if (value instanceof Pose3D) {
                return (Pose3D) value;
            }
        } catch (Throwable ignored) {
        }
        for (String method : resultMethods) {
            try {
                Object value = result.getClass().getMethod(method).invoke(result);
                if (value instanceof Pose3D) {
                    return (Pose3D) value;
                }
            } catch (Throwable ignored) {
            }
        }
        return null;
    }

    private void updateRawMt2Debug(LlXySample sample) {
        if (sample == null) return;
        lastMt2Xmeters = sample.xMeters;
        lastMt2Ymeters = sample.yMeters;
        lastMt2Xin = sample.xIn;
        lastMt2Yin = sample.yIn;
    }

    private static class LlXySample {
        private double xMeters = Double.NaN;
        private double yMeters = Double.NaN;
        private double xIn = Double.NaN;
        private double yIn = Double.NaN;
        private double tx = Double.NaN;
        private double ty = Double.NaN;
        private double fx = Double.NaN;
        private double fy = Double.NaN;
    }

    private void updateOdometryDebugLine() {
        String ticks = String.format(Locale.US, "%d/%d/%d/%d",
                lastWheelDeltaTicks[0],
                lastWheelDeltaTicks[1],
                lastWheelDeltaTicks[2],
                lastWheelDeltaTicks[3]);
        String raw = String.format(Locale.US, "%.2f/%.2f/%.2f/%.2f",
                lastWheelDeltaRaw[0],
                lastWheelDeltaRaw[1],
                lastWheelDeltaRaw[2],
                lastWheelDeltaRaw[3]);
        String scaled = String.format(Locale.US, "%.2f/%.2f/%.2f/%.2f",
                lastWheelDeltaScaled[0],
                lastWheelDeltaScaled[1],
                lastWheelDeltaScaled[2],
                lastWheelDeltaScaled[3]);
        odometryDebugLine = String.format(Locale.US,
                "OdoDbg ticks=%s raw=%s in scaled=%s in pose=%.1f,%.1f h=%.1f",
                ticks,
                raw,
                scaled,
                pose.x,
                pose.y,
                pose.headingDeg);
    }

    private String formatPoint(FieldPose point) {
        if (point == null) return "--,--";
        return String.format(Locale.US, "%.1f,%.1f", point.x, point.y);
    }

    private boolean isWithinFieldBounds(double x, double y, double bound) {
        return Math.abs(x) <= bound && Math.abs(y) <= bound;
    }

    private double boundsMaxIn() {
        double margin = VisionConfig.LimelightFusion.BOUNDS_MARGIN_IN;
        if (margin < 0.0) margin = 0.0;
        double maxMargin = VisionConfig.LimelightFusion.FIELD_HALF_IN - 1.0;
        if (margin > maxMargin) margin = maxMargin;
        return VisionConfig.LimelightFusion.FIELD_HALF_IN - margin;
    }

    private String formatPair(Double x, Double y) {
        if (x == null || y == null) return "--,--";
        return String.format(Locale.US, "%.1f,%.1f", x, y);
    }

    private String formatIds(int[] ids) {
        if (ids == null || ids.length == 0) return "--";
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < ids.length; i++) {
            if (i > 0) sb.append(",");
            sb.append(ids[i]);
        }
        return sb.toString();
    }

    private boolean isTagAllowedForLocalization(Integer tagId, int[] allowed) {
        if (tagId == null || allowed == null || allowed.length == 0) return false;
        for (int id : allowed) {
            if (tagId == id) return true;
        }
        return false;
    }

    private int[] filterIds(int[] base, int[] excluded) {
        if (base == null || base.length == 0) return new int[0];
        if (excluded == null || excluded.length == 0) return Arrays.copyOf(base, base.length);
        int[] tmp = new int[base.length];
        int count = 0;
        for (int id : base) {
            boolean skip = false;
            for (int ex : excluded) {
                if (id == ex) {
                    skip = true;
                    break;
                }
            }
            if (!skip) {
                tmp[count++] = id;
            }
        }
        return Arrays.copyOf(tmp, count);
    }

    private int countFiducials(LLResult result) {
        if (result == null) return 0;
        Object list = readList(result, "getFiducialResults");
        if (list instanceof Iterable) {
            int count = 0;
            for (Object ignored : (Iterable<?>) list) {
                count++;
            }
            return count;
        }
        int[] ids = readIntArray(result, "getFiducialIds", "getTargetIds", "getTidList");
        if (ids != null) return ids.length;
        double[] idsDouble = readDoubleArray(result, "getFiducialIds", "getTargetIds", "getTidList");
        return idsDouble != null ? idsDouble.length : 0;
    }

    private Integer readPrimaryTagId(LLResult result) {
        if (result == null) return null;
        Integer direct = readSingleId(result, "getFiducialId", "getTid", "getTargetId");
        if (direct != null) return direct;
        Object list = readList(result, "getFiducialResults");
        if (list instanceof Iterable) {
            for (Object entry : (Iterable<?>) list) {
                Integer id = readSingleId(entry, "getFiducialId", "getTid", "getTargetId");
                if (id != null) return id;
            }
        }
        return null;
    }

    private Object readList(Object owner, String... methods) {
        for (String method : methods) {
            try {
                Object value = owner.getClass().getMethod(method).invoke(owner);
                if (value != null) {
                    return value;
                }
            } catch (Throwable ignored) { }
        }
        return null;
    }

    private Integer readSingleId(Object owner, String... methods) {
        for (String method : methods) {
            try {
                Object value = owner.getClass().getMethod(method).invoke(owner);
                if (value instanceof Number) {
                    return ((Number) value).intValue();
                }
            } catch (Throwable ignored) { }
        }
        return null;
    }

    private int[] readVisibleFiducialIds(LLResult result) {
        List<Integer> ids = new java.util.ArrayList<>();
        Object list = readList(result, "getFiducialResults");
        if (list instanceof Iterable) {
            for (Object entry : (Iterable<?>) list) {
                Integer id = readSingleId(entry, "getFiducialId", "getTid", "getTargetId");
                if (id != null) {
                    ids.add(id);
                }
            }
        }
        if (ids.isEmpty()) {
            int[] raw = readIntArray(result, "getFiducialIds", "getTargetIds", "getTidList");
            if (raw != null) {
                for (int id : raw) {
                    ids.add(id);
                }
            } else {
                double[] rawDouble = readDoubleArray(result, "getFiducialIds", "getTargetIds", "getTidList");
                if (rawDouble != null) {
                    for (double id : rawDouble) {
                        ids.add((int) Math.round(id));
                    }
                }
            }
        }
        if (ids.isEmpty()) return new int[0];
        java.util.Set<Integer> unique = new java.util.HashSet<>(ids);
        int[] out = new int[unique.size()];
        int idx = 0;
        for (Integer id : unique) {
            out[idx++] = id;
        }
        Arrays.sort(out);
        return out;
    }

    private int[] filterObeliskIds(int[] ids) {
        if (ids == null || ids.length == 0) return new int[0];
        int[] tmp = new int[ids.length];
        int count = 0;
        for (int id : ids) {
            if (VisionConfig.isObeliskTagId(id)) {
                tmp[count++] = id;
            }
        }
        return Arrays.copyOf(tmp, count);
    }

    private int[] readIntArray(Object owner, String... methods) {
        for (String method : methods) {
            try {
                Object value = owner.getClass().getMethod(method).invoke(owner);
                if (value instanceof int[]) {
                    return (int[]) value;
                }
            } catch (Throwable ignored) { }
        }
        return null;
    }

    private double[] readDoubleArray(Object owner, String... methods) {
        for (String method : methods) {
            try {
                Object value = owner.getClass().getMethod(method).invoke(owner);
                if (value instanceof double[]) {
                    return (double[]) value;
                }
            } catch (Throwable ignored) { }
        }
        return null;
    }

    private Long readTimestampMs(LLResult result) {
        try {
            Object seconds = result.getClass().getMethod("getTimestampSeconds").invoke(result);
            if (seconds instanceof Number) {
                return (long) (((Number) seconds).doubleValue() * 1000.0);
            }
        } catch (Throwable ignored) { }

        try {
            Object millis = result.getClass().getMethod("getTimestampMs").invoke(result);
            if (millis instanceof Number) {
                return ((Number) millis).longValue();
            }
        } catch (Throwable ignored) { }

        return null;
    }

    private FieldPose lowPass(FieldPose current, FieldPose target, double alpha) {
        double a = clampValue(alpha, 0.0, 1.0);
        return new FieldPose(
                current.x + (target.x - current.x) * a,
                current.y + (target.y - current.y) * a,
                normHeading(current.headingDeg + (target.headingDeg - current.headingDeg) * a)
        );
    }

    /**
     * Normalize IMU heading into the shared odometry frame where 0° faces the
     * target wall (+Y) and positive angles turn counter-clockwise toward +X.
     */
    private double normalizeHeading(double rawHeadingDeg) {
        double shifted = rawHeadingDeg + OdometryConfig.IMU_HEADING_OFFSET_DEG;
        double fieldFrame = shifted; // align IMU basis (0° = +X) to odometry basis (0° = +Y)
        return normHeading(fieldFrame);
    }

    private double normHeading(double h) {
        double v = h % 360.0;
        if (v < -180) v += 360.0;
        if (v > 180) v -= 360.0;
        return v;
    }

    private double[] clampMagnitude(double x, double y, double maxMag) {
        double mag = hypot(x, y);
        if (mag <= maxMag || mag < 1e-6) return new double[]{x, y};
        double scale = maxMag / mag;
        return new double[]{x * scale, y * scale};
    }

    private double clampValue(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
