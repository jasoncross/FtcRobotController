package org.firstinspires.ftc.teamcode.utils;

/*
 * FILE: ObeliskSignal.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/
 *
 * PURPOSE:
 *   Shared, in-memory store for the DECODE obelisk AprilTag signal.
 *   The field crew calls this the 'puzzle tower' that displays one of three tags:
 *     - Tag 21 → GPP (Green, Purple, Purple)
 *     - Tag 22 → PGP
 *     - Tag 23 → PPG
 *
 *   Once any OpMode sees one of these tags, this class latches the corresponding
 *   optimal ball order so ALL OpModes (Auto + TeleOp) can read and display it.
 *
 * API:
 *   - updateFromTagId(id): Call when you see one of the 3 tag IDs.
 *   - get(): Returns the latched value as an enum.
 *   - clear(): Resets to UNKNOWN (useful for testing between matches).
 *   - getDisplay(): Human-friendly string for Telemetry first line.
 *
 * THREADING:
 *   Uses simple volatile fields; multi-hub OpMode threads will see updates quickly.
 */
public final class ObeliskSignal {
    public enum Order { GPP, PGP, PPG, UNKNOWN }

    private static volatile Order current = Order.UNKNOWN;
    private static volatile long lastUpdateMs = 0L;
    private static volatile int lastTagId = -1;

    private ObeliskSignal() {}

    /** Update the current obelisk order based on the AprilTag ID */
    public static void updateFromTagId(int id) {
        switch (id) {
            case 21: set(Order.GPP); break;
            case 22: set(Order.PGP); break;
            case 23: set(Order.PPG); break;
            default: /* ignore other tags */ break;
        }
        lastTagId = id;
    }

    /** Explicitly set an order */
    public static void set(Order order) {
        if (order == null) return;
        current = order;
        lastUpdateMs = System.currentTimeMillis();
        if (order == Order.UNKNOWN) lastTagId = -1;
    }

    /** Returns the current latched order */
    public static Order get() { return current; }

    /** Returns the last AprilTag ID that updated the latch; -1 when unknown */
    public static int getLastTagId() { return lastTagId; }

    /** Returns the age of the last update in milliseconds */
    public static long ageMs() {
        if (lastUpdateMs == 0L) return Long.MAX_VALUE;
        return Math.max(0L, System.currentTimeMillis() - lastUpdateMs);
    }

    /** Resets the latch to UNKNOWN */
    public static void clear() { current = Order.UNKNOWN; lastUpdateMs = 0L; lastTagId = -1; }

    /** Returns a readable string for telemetry */
    public static String getDisplay() {
        if (current == Order.UNKNOWN) return "Obelisk: ---";
        long ageS = (lastUpdateMs == 0L)
                ? 0L
                : Math.max(0L, (System.currentTimeMillis() - lastUpdateMs) / 1000L);
        return "Obelisk: " + current + (ageS > 0 ? " (" + ageS + "s ago)" : "");
    }
}
