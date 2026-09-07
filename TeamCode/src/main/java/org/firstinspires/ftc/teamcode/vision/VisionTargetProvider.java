package org.firstinspires.ftc.teamcode.vision;

import java.util.List;

/** Common webcam/Limelight interface without alliance, scoring, or field-map assumptions. */
public interface VisionTargetProvider extends AutoCloseable {
    void update();
    List<TargetObservation> getTargets();
    String getStatus();
    default TargetObservation getTarget(int id) {
        if (id < 0) return null;
        for (TargetObservation target : getTargets()) if (target.id == id) return target;
        return null;
    }
    @Override void close();
}
