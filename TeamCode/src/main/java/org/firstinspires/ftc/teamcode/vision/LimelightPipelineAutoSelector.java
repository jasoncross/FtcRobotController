package org.firstinspires.ftc.teamcode.vision;

import android.content.Context;
import android.content.SharedPreferences;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.config.LimelightPipelineAutoSelectConfig;
import org.firstinspires.ftc.teamcode.config.VisionConfig;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.function.Supplier;

/*
 * FILE: LimelightPipelineAutoSelector.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/
 *
 * PURPOSE
 *   - Evaluate a list of Limelight pipelines during INIT, score them based on
 *     AprilTag hit counts, and lock the best profile for the remainder of the
 *     match without blocking the OpMode loop (continues after START if needed).
 *
 * SELECTION SUMMARY
 *   - Goal and opposing goal hits are tallied per pipeline (obelisk ignored).
 *   - Goal hits meeting PIPELINE_MIN_GOAL_HITS rank highest.
 *   - Opposing hits meeting PIPELINE_MIN_OPP_GOAL_HITS rank next.
 *   - Otherwise rank is 0 (failure fallback to pipeline 0).
 *
 * CHANGES (2025-12-28): Added non-blocking INIT pipeline auto-selection with
 *                       tunable profiles, precedence scoring, and telemetry
 *                       helpers shared by TeleOp and Auto.
 * CHANGES (2025-12-28): Removed obelisk tags from pipeline scoring so only
 *                       alliance/opposing goal tags affect selection.
 * CHANGES (2025-12-28): Added hit-count qualification logic, ensured Limelight
 *                       start before sampling, and allowed selection to
 *                       continue after START until completion/timeout.
 * CHANGES (2025-12-28): Adjusted running telemetry to report the testing
 *                       pipeline index with post-start continuation.
 * CHANGES (2025-12-28): Confirmed START no longer forces selection finalize
 *                       or fallback so updates continue post-start.
 * CHANGES (2025-12-28): Removed START-based resets so INIT selection can
 *                       continue into START without forced timeouts.
 * CHANGES (2025-12-28): Added tunable fallback pipeline index instead of
 *                       hard-coding pipeline 0.
 * CHANGES (2025-12-28): Added last-known-good memory fallback with optional
 *                       SharedPreferences persistence and telemetry support.
 */
public class LimelightPipelineAutoSelector {
    private static final int RANK_NONE = 0;
    private static final int RANK_OPPOSING_GOAL = 1;
    private static final int RANK_GOAL = 2;

    private static final String DEFAULT_PROFILE_NAME = "Meet Gym";

    private final Limelight3A limelight;
    private final LimelightTargetProvider provider;
    private final Supplier<Alliance> allianceSupplier;
    private final List<PipelineProfile> profiles;
    private boolean locked = false;
    private boolean fallbackFailure = false;
    private String failureReason = null;
    private boolean memoryFallbackUsed = false;
    private String memoryFallbackReason = null;
    private Integer selectedPipeline = null;
    private String selectedDescription = null;
    private Integer activePipeline = null;

    private Stage stage = Stage.IDLE;
    private int currentProfileIdx = 0;
    private int bestRankOverall = 0;
    private final List<Integer> bestRankPipelines = new ArrayList<>();
    private int goalHitsThisPipeline = 0;
    private int oppHitsThisPipeline = 0;
    private int samplesTaken = 0;
    private long selectionStartMs = 0L;
    private long pipelineSwitchMs = 0L;
    private long lastSampleMs = 0L;
    private Long opModeStartedMs = null;

    private enum Stage {
        IDLE,
        SETTLING,
        SAMPLING,
        COMPLETE
    }

    public LimelightPipelineAutoSelector(Limelight3A limelight,
                                          LimelightTargetProvider provider,
                                          Supplier<Alliance> allianceSupplier) {
        this.limelight = limelight;
        this.provider = provider;
        this.allianceSupplier = allianceSupplier;
        this.profiles = parseProfiles(LimelightPipelineAutoSelectConfig.LIMELIGHT_PIPELINE_PROFILES);
        LastKnownGood.ensureLoaded();
    }

    public boolean isEnabled() {
        return LimelightPipelineAutoSelectConfig.ENABLE_LIMELIGHT_PIPELINE_AUTOSELECT;
    }

    public boolean isLocked() {
        return locked;
    }

    public boolean isFallbackFailure() {
        return fallbackFailure;
    }

    public String getFailureReason() {
        return failureReason;
    }

    public Integer getSelectedPipeline() {
        return selectedPipeline;
    }

    public String getSelectedDescription() {
        return selectedDescription;
    }

    public void update() {
        if (!isEnabled() || locked) {
            return;
        }
        if (limelight == null || provider == null) {
            lockFallback("no_limelight");
            return;
        }

        long now = System.currentTimeMillis();
        if (stage == Stage.IDLE) {
            provider.ensureStarted();
            selectionStartMs = now;
            currentProfileIdx = 0;
            bestRankOverall = 0;
            bestRankPipelines.clear();
            beginProfile(now);
            return;
        }

        if (LimelightPipelineAutoSelectConfig.PIPELINE_MAX_SELECTION_MS > 0
                && (now - selectionStartMs) >= LimelightPipelineAutoSelectConfig.PIPELINE_MAX_SELECTION_MS) {
            finalizeSelection("timeout");
            return;
        }

        PipelineProfile profile = profiles.get(currentProfileIdx);

        if (stage == Stage.SETTLING) {
            if ((now - pipelineSwitchMs) >= LimelightPipelineAutoSelectConfig.PIPELINE_SETTLE_MS) {
                stage = Stage.SAMPLING;
                lastSampleMs = 0L;
            }
            return;
        }

        if (stage == Stage.SAMPLING) {
            if (samplesTaken >= LimelightPipelineAutoSelectConfig.PIPELINE_SAMPLE_COUNT) {
                finalizeProfile(profile);
                if (currentProfileIdx + 1 < profiles.size()) {
                    currentProfileIdx++;
                    beginProfile(now);
                } else {
                    finalizeSelection(null);
                }
                return;
            }

            if (lastSampleMs == 0L || (now - lastSampleMs) >= LimelightPipelineAutoSelectConfig.PIPELINE_SAMPLE_INTERVAL_MS) {
                tallyHits(provider.getVisibleTagIds());
                samplesTaken++;
                lastSampleMs = now;
            }
        }
    }

    public String getProfileLine() {
        PipelineProfile resolved = resolveTelemetryProfile();
        if (resolved == null) {
            return "LL Profile: -- - --";
        }
        return String.format(Locale.US, "LL Profile: %d - %s", resolved.pipelineIndex, resolved.description);
    }

    public String getFallbackBannerLine() {
        if (!fallbackFailure) return null;
        int fallbackIndex = resolveFallbackPipelineIndex();
        PipelineProfile fallback = resolveProfileForIndex(fallbackIndex);
        String desc = (fallback != null) ? fallback.description : DEFAULT_PROFILE_NAME;
        String reason = (failureReason == null || failureReason.isEmpty()) ? "no_tags" : failureReason;
        return String.format(Locale.US,
                "LL AUTOSELECT FALLBACK -> %d - %s (reason=%s)",
                fallbackIndex,
                desc,
                reason);
    }

    public String getMemoryFallbackLine() {
        if (!memoryFallbackUsed) return null;
        int pipelineIndex = (selectedPipeline != null) ? selectedPipeline : resolveFallbackPipelineIndex();
        String desc = (selectedDescription != null && !selectedDescription.isEmpty())
                ? selectedDescription
                : resolveLastKnownGoodDescription(pipelineIndex, LastKnownGood.getDescription());
        String reason = (memoryFallbackReason == null || memoryFallbackReason.isEmpty()) ? "no_tags" : memoryFallbackReason;
        return String.format(Locale.US,
                "LL AUTOSELECT: MEMORY FALLBACK -> %d - %s (reason=%s)",
                pipelineIndex,
                desc,
                reason);
    }

    public void lockToSelectedPipeline() {
        if (selectedPipeline != null) {
            applyPipeline(selectedPipeline);
        }
    }

    public void notifyOpModeStarted() {
        if (opModeStartedMs == null) {
            opModeStartedMs = System.currentTimeMillis();
        }
    }

    public String getRunningStatusLine() {
        if (locked || stage == Stage.COMPLETE || stage == Stage.IDLE) {
            return null;
        }
        int profileIndex = resolveCurrentProfileIndex();
        return String.format(Locale.US,
                "LL AUTOSELECT: RUNNING (stage=%s testing=%d)",
                stage.name(),
                profileIndex);
    }

    private int resolveFallbackPipelineIndex() {
        int configured = LimelightPipelineAutoSelectConfig.PIPELINE_FALLBACK_INDEX;
        return configured >= 0 ? configured : 0;
    }

    private void beginProfile(long now) {
        PipelineProfile profile = profiles.get(currentProfileIdx);
        applyPipeline(profile.pipelineIndex);
        activePipeline = profile.pipelineIndex;
        pipelineSwitchMs = now;
        samplesTaken = 0;
        goalHitsThisPipeline = 0;
        oppHitsThisPipeline = 0;
        stage = Stage.SETTLING;
    }

    private void finalizeProfile(PipelineProfile profile) {
        int rank = computeRank(goalHitsThisPipeline, oppHitsThisPipeline);
        if (rank > bestRankOverall) {
            bestRankOverall = rank;
            bestRankPipelines.clear();
            bestRankPipelines.add(profile.pipelineIndex);
        } else if (rank == bestRankOverall) {
            bestRankPipelines.add(profile.pipelineIndex);
        }
    }

    private void finalizeSelection(String reasonOverride) {
        if (bestRankOverall == RANK_NONE) {
            String reason = (reasonOverride != null) ? reasonOverride : "no_tags";
            lockFallback(reason);
            return;
        }

        int chosen = chooseWinningPipeline(bestRankPipelines);
        PipelineProfile profile = resolveProfileForIndex(chosen);
        selectedPipeline = chosen;
        selectedDescription = (profile != null) ? profile.description : DEFAULT_PROFILE_NAME;
        locked = true;
        fallbackFailure = false;
        failureReason = null;
        memoryFallbackUsed = false;
        memoryFallbackReason = null;
        stage = Stage.COMPLETE;
        applyPipeline(chosen);
        LastKnownGood.setLastGood(chosen, selectedDescription);
    }

    private int chooseWinningPipeline(List<Integer> winners) {
        if (winners == null || winners.isEmpty()) {
            return 0;
        }
        if (winners.size() == 1) {
            return winners.get(0);
        }
        if (winners.contains(0)) {
            return 0;
        }
        int min = winners.get(0);
        for (int idx : winners) {
            if (idx < min) min = idx;
        }
        return min;
    }

    private void lockFallback(String reason) {
        boolean allowMemory = true;
        if ("timeout".equals(reason)) {
            allowMemory = LimelightPipelineAutoSelectConfig.PIPELINE_USE_LAST_GOOD_ON_TIMEOUT;
        }
        boolean useMemory = allowMemory
                && LimelightPipelineAutoSelectConfig.PIPELINE_REMEMBER_LAST_GOOD_ENABLED
                && LastKnownGood.hasLastGood();
        if (useMemory) {
            Integer lastGoodIndex = LastKnownGood.getIndex();
            if (lastGoodIndex != null && lastGoodIndex >= 0) {
                locked = true;
                fallbackFailure = false;
                failureReason = null;
                memoryFallbackUsed = true;
                memoryFallbackReason = reason;
                selectedPipeline = lastGoodIndex;
                selectedDescription = resolveLastKnownGoodDescription(lastGoodIndex, LastKnownGood.getDescription());
                stage = Stage.COMPLETE;
                applyPipeline(lastGoodIndex);
                return;
            }
        }
        locked = true;
        fallbackFailure = true;
        failureReason = reason;
        memoryFallbackUsed = false;
        memoryFallbackReason = null;
        int fallbackIndex = resolveFallbackPipelineIndex();
        selectedPipeline = fallbackIndex;
        PipelineProfile profile = resolveProfileForIndex(fallbackIndex);
        selectedDescription = (profile != null) ? profile.description : DEFAULT_PROFILE_NAME;
        stage = Stage.COMPLETE;
        applyPipeline(fallbackIndex);
    }

    private void applyPipeline(int pipelineIndex) {
        if (provider != null) {
            provider.setPipelineIndex(pipelineIndex);
        } else if (limelight != null) {
            try {
                limelight.pipelineSwitch(pipelineIndex);
            } catch (Throwable ignored) { }
        }
        activePipeline = pipelineIndex;
    }

    private PipelineProfile resolveTelemetryProfile() {
        if (selectedPipeline != null) {
            PipelineProfile profile = resolveProfileForIndex(selectedPipeline);
            return (profile != null) ? profile : new PipelineProfile(selectedPipeline, DEFAULT_PROFILE_NAME);
        }
        Integer pipelineIdx = activePipeline;
        if (pipelineIdx == null && provider != null) {
            pipelineIdx = provider.getActivePipelineIndex();
        }
        if (pipelineIdx == null) {
            pipelineIdx = resolveFallbackPipelineIndex();
        }
        PipelineProfile profile = resolveProfileForIndex(pipelineIdx);
        return (profile != null) ? profile : new PipelineProfile(pipelineIdx, DEFAULT_PROFILE_NAME);
    }

    private void tallyHits(List<Integer> ids) {
        if (ids == null || ids.isEmpty()) return;
        Alliance alliance = (allianceSupplier != null) ? allianceSupplier.get() : Alliance.BLUE;
        int goalId = VisionConfig.goalTagIdForAlliance(alliance);
        int opposingGoal = (goalId == VisionConfig.GOAL_TAG_BLUE)
                ? VisionConfig.GOAL_TAG_RED
                : VisionConfig.GOAL_TAG_BLUE;

        if (ids.contains(goalId)) {
            goalHitsThisPipeline++;
        }
        if (!LimelightPipelineAutoSelectConfig.PIPELINE_REQUIRE_GOAL_ONLY && ids.contains(opposingGoal)) {
            oppHitsThisPipeline++;
        }
    }

    private int computeRank(int goalHits, int oppHits) {
        if (goalHits >= LimelightPipelineAutoSelectConfig.PIPELINE_MIN_GOAL_HITS) {
            return RANK_GOAL;
        }
        if (!LimelightPipelineAutoSelectConfig.PIPELINE_REQUIRE_GOAL_ONLY
                && oppHits >= LimelightPipelineAutoSelectConfig.PIPELINE_MIN_OPP_GOAL_HITS) {
            return RANK_OPPOSING_GOAL;
        }
        return RANK_NONE;
    }

    private int resolveCurrentProfileIndex() {
        PipelineProfile profile = profiles.get(currentProfileIdx);
        if (profile != null) {
            return profile.pipelineIndex;
        }
        if (activePipeline != null) {
            return activePipeline;
        }
        return 0;
    }

    private List<PipelineProfile> parseProfiles(String raw) {
        if (raw == null) raw = "";
        List<PipelineProfile> parsed = new ArrayList<>();
        String[] entries = raw.split(";");
        for (String entry : entries) {
            if (entry == null) continue;
            String trimmed = entry.trim();
            if (trimmed.isEmpty()) continue;
            String[] parts = trimmed.split(":", 2);
            if (parts.length < 2) continue;
            String idxStr = parts[0].trim();
            String desc = parts[1].trim();
            if (idxStr.isEmpty() || desc.isEmpty()) continue;
            try {
                int idx = Integer.parseInt(idxStr);
                if (idx < 0) continue;
                parsed.add(new PipelineProfile(idx, desc));
            } catch (NumberFormatException ignored) { }
        }

        if (parsed.isEmpty()) {
            parsed.add(new PipelineProfile(0, DEFAULT_PROFILE_NAME));
        }

        return Collections.unmodifiableList(parsed);
    }

    private PipelineProfile resolveProfileForIndex(int pipelineIndex) {
        for (PipelineProfile profile : profiles) {
            if (profile.pipelineIndex == pipelineIndex) {
                return profile;
            }
        }
        return null;
    }

    private String resolveLastKnownGoodDescription(int pipelineIndex, String persistedDescription) {
        if (persistedDescription != null && !persistedDescription.trim().isEmpty()) {
            return persistedDescription;
        }
        PipelineProfile profile = resolveProfileForIndex(pipelineIndex);
        if (profile != null) {
            return profile.description;
        }
        return DEFAULT_PROFILE_NAME;
    }

    private static final class LastKnownGood {
        private static final String PREF_NAME = "limelight_pipeline_autoselect";
        private static final String KEY_INDEX = "lastGoodPipelineIndex";
        private static final String KEY_DESCRIPTION = "lastGoodPipelineDescription";
        private static final String KEY_TIMESTAMP = "lastGoodTimestampMs";
        private static final int UNKNOWN_INDEX = -1;

        private static boolean loaded = false;
        private static Integer cachedIndex = null;
        private static String cachedDescription = null;
        private static Long cachedTimestampMs = null;

        private static void ensureLoaded() {
            if (loaded) return;
            loaded = true;
            if (!LimelightPipelineAutoSelectConfig.PIPELINE_PERSIST_LAST_GOOD_ENABLED) {
                return;
            }
            Context context = AppUtil.getDefContext();
            if (context == null) {
                return;
            }
            SharedPreferences prefs = context.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);
            if (!prefs.contains(KEY_INDEX)) {
                return;
            }
            int index = prefs.getInt(KEY_INDEX, UNKNOWN_INDEX);
            if (index >= 0) {
                cachedIndex = index;
            }
            cachedDescription = prefs.getString(KEY_DESCRIPTION, null);
            long ts = prefs.getLong(KEY_TIMESTAMP, -1L);
            if (ts > 0L) {
                cachedTimestampMs = ts;
            }
        }

        private static boolean hasLastGood() {
            ensureLoaded();
            return cachedIndex != null && cachedIndex >= 0;
        }

        private static Integer getIndex() {
            ensureLoaded();
            return cachedIndex;
        }

        private static String getDescription() {
            ensureLoaded();
            return cachedDescription;
        }

        private static void setLastGood(int index, String description) {
            if (index < 0) return;
            cachedIndex = index;
            cachedDescription = description;
            cachedTimestampMs = System.currentTimeMillis();
            loaded = true;
            if (!LimelightPipelineAutoSelectConfig.PIPELINE_PERSIST_LAST_GOOD_ENABLED) {
                return;
            }
            Context context = AppUtil.getDefContext();
            if (context == null) {
                return;
            }
            SharedPreferences.Editor editor = context.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE).edit();
            editor.putInt(KEY_INDEX, index);
            if (description != null) {
                editor.putString(KEY_DESCRIPTION, description);
            } else {
                editor.remove(KEY_DESCRIPTION);
            }
            if (cachedTimestampMs != null) {
                editor.putLong(KEY_TIMESTAMP, cachedTimestampMs);
            } else {
                editor.remove(KEY_TIMESTAMP);
            }
            editor.apply();
        }

        private static void clear() {
            cachedIndex = null;
            cachedDescription = null;
            cachedTimestampMs = null;
            loaded = true;
            if (!LimelightPipelineAutoSelectConfig.PIPELINE_PERSIST_LAST_GOOD_ENABLED) {
                return;
            }
            Context context = AppUtil.getDefContext();
            if (context == null) {
                return;
            }
            SharedPreferences.Editor editor = context.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE).edit();
            editor.remove(KEY_INDEX);
            editor.remove(KEY_DESCRIPTION);
            editor.remove(KEY_TIMESTAMP);
            editor.apply();
        }
    }

    private static final class PipelineProfile {
        private final int pipelineIndex;
        private final String description;

        private PipelineProfile(int pipelineIndex, String description) {
            this.pipelineIndex = pipelineIndex;
            this.description = description;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (!(obj instanceof PipelineProfile)) return false;
            PipelineProfile other = (PipelineProfile) obj;
            return pipelineIndex == other.pipelineIndex && Objects.equals(description, other.description);
        }

        @Override
        public int hashCode() {
            return Objects.hash(pipelineIndex, description);
        }
    }
}
