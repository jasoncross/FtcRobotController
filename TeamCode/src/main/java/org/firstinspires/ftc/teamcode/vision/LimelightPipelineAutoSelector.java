package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.config.LimelightPipelineAutoSelectConfig;
import org.firstinspires.ftc.teamcode.config.VisionConfig;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

/*
 * FILE: LimelightPipelineAutoSelector.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/
 *
 * PURPOSE
 *   - Evaluate a list of Limelight pipelines during INIT, score them based on
 *     AprilTag precedence, and lock the best profile for the remainder of the
 *     match without blocking the OpMode loop.
 *
 * SELECTION SUMMARY
 *   - Goal tag for the current alliance has highest precedence.
 *   - Opposing goal tag ranks next.
 *   - Obelisk tags (21–23) rank next.
 *   - No relevant tag yields a rank of 0 (failure fallback to pipeline 0).
 *
 * CHANGES (2025-12-28): Added non-blocking INIT pipeline auto-selection with
 *                       tunable profiles, precedence scoring, and telemetry
 *                       helpers shared by TeleOp and Auto.
 */
public class LimelightPipelineAutoSelector {
    private static final int RANK_NONE = 0;
    private static final int RANK_OBELISK = 1;
    private static final int RANK_OPPOSING_GOAL = 2;
    private static final int RANK_GOAL = 3;

    private static final String DEFAULT_PROFILE_NAME = "Meet Gym";

    private final Limelight3A limelight;
    private final LimelightTargetProvider provider;
    private final Supplier<Alliance> allianceSupplier;
    private final List<PipelineProfile> profiles;
    private final Map<Integer, Integer> bestRankByPipeline = new HashMap<>();

    private boolean locked = false;
    private boolean fallbackFailure = false;
    private String failureReason = null;
    private Integer selectedPipeline = null;
    private String selectedDescription = null;
    private Integer activePipeline = null;

    private Stage stage = Stage.IDLE;
    private int currentProfileIdx = 0;
    private int bestRankOverall = 0;
    private final List<Integer> bestRankPipelines = new ArrayList<>();
    private int bestRankThisPipeline = 0;
    private int samplesTaken = 0;
    private long selectionStartMs = 0L;
    private long pipelineSwitchMs = 0L;
    private long lastSampleMs = 0L;

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
            selectionStartMs = now;
            currentProfileIdx = 0;
            bestRankOverall = 0;
            bestRankPipelines.clear();
            bestRankByPipeline.clear();
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
                int rank = rankVisibleTags(provider.getVisibleTagIds());
                if (rank > bestRankThisPipeline) {
                    bestRankThisPipeline = rank;
                }
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
        PipelineProfile fallback = resolveProfileForIndex(0);
        String desc = (fallback != null) ? fallback.description : DEFAULT_PROFILE_NAME;
        String reason = (failureReason == null || failureReason.isEmpty()) ? "no_tags" : failureReason;
        return String.format(Locale.US,
                "LL AUTOSELECT FALLBACK -> 0 - %s (reason=%s)",
                desc,
                reason);
    }

    public void finalizeOnStart() {
        if (!isEnabled() || locked) {
            return;
        }
        if (stage != Stage.IDLE && stage != Stage.COMPLETE) {
            PipelineProfile profile = profiles.get(currentProfileIdx);
            if (profile != null) {
                bestRankByPipeline.put(profile.pipelineIndex, bestRankThisPipeline);
                if (bestRankThisPipeline > bestRankOverall) {
                    bestRankOverall = bestRankThisPipeline;
                    bestRankPipelines.clear();
                    bestRankPipelines.add(profile.pipelineIndex);
                } else if (bestRankThisPipeline == bestRankOverall) {
                    if (!bestRankPipelines.contains(profile.pipelineIndex)) {
                        bestRankPipelines.add(profile.pipelineIndex);
                    }
                }
            }
        }
        finalizeSelection("timeout");
    }

    public void lockToSelectedPipeline() {
        if (selectedPipeline != null) {
            applyPipeline(selectedPipeline);
        }
    }

    private void beginProfile(long now) {
        PipelineProfile profile = profiles.get(currentProfileIdx);
        applyPipeline(profile.pipelineIndex);
        activePipeline = profile.pipelineIndex;
        pipelineSwitchMs = now;
        samplesTaken = 0;
        bestRankThisPipeline = 0;
        stage = Stage.SETTLING;
    }

    private void finalizeProfile(PipelineProfile profile) {
        bestRankByPipeline.put(profile.pipelineIndex, bestRankThisPipeline);
        if (bestRankThisPipeline > bestRankOverall) {
            bestRankOverall = bestRankThisPipeline;
            bestRankPipelines.clear();
            bestRankPipelines.add(profile.pipelineIndex);
        } else if (bestRankThisPipeline == bestRankOverall) {
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
        stage = Stage.COMPLETE;
        applyPipeline(chosen);
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
        locked = true;
        fallbackFailure = true;
        failureReason = reason;
        selectedPipeline = 0;
        PipelineProfile profile = resolveProfileForIndex(0);
        selectedDescription = (profile != null) ? profile.description : DEFAULT_PROFILE_NAME;
        stage = Stage.COMPLETE;
        applyPipeline(0);
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
            pipelineIdx = 0;
        }
        PipelineProfile profile = resolveProfileForIndex(pipelineIdx);
        return (profile != null) ? profile : new PipelineProfile(pipelineIdx, DEFAULT_PROFILE_NAME);
    }

    private int rankVisibleTags(List<Integer> ids) {
        if (ids == null || ids.isEmpty()) return RANK_NONE;
        Alliance alliance = (allianceSupplier != null) ? allianceSupplier.get() : Alliance.BLUE;
        int goalId = VisionConfig.goalTagIdForAlliance(alliance);
        int opposingGoal = (goalId == VisionConfig.GOAL_TAG_BLUE)
                ? VisionConfig.GOAL_TAG_RED
                : VisionConfig.GOAL_TAG_BLUE;

        boolean hasGoal = ids.contains(goalId);
        if (hasGoal) return RANK_GOAL;

        if (ids.contains(opposingGoal)) {
            return RANK_OPPOSING_GOAL;
        }

        for (int id : ids) {
            if (id >= 21 && id <= 23) {
                return RANK_OBELISK;
            }
        }

        return RANK_NONE;
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
