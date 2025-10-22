package org.firstinspires.ftc.teamcode.input;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

/*
 * ============================================================================
 * FILE: ControllerBindings.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/input/
 * AUTHOR: Indianola Robotics (Octobots)
 *
 * PURPOSE:
 *   Centralize ALL controller mappings (Gamepad 1 & Gamepad 2) so assignments
 *   can be changed in ONE place without touching TeleOp code. Supports:
 *     - Press (edge-detected once per press)
 *     - Hold (runs every loop while down)
 *     - Toggle (flip state with press, with on/off callbacks)
 *     - Trigger Axis (LT/RT 0..1 each loop)
 *     - Optional M1/M2 rear paddles via pluggable readers
 *
 * CURRENTLY-BOUND FUNCTIONS  (INTENTIONALLY LIMITED to what’s actually mapped)
 *   Intake Toggle .......... intake.toggle()                  (LB on G1/G2)
 *   Feed Once .............. feed.feedOnceBlocking()          (RB on G1/G2)
 *   Aim Assist Toggle ...... aimEnabled true/false            (R_STICK_BTN on G1 ONLY)
 *   Manual Speed Mode ...... manualSpeedMode true/false       (Y on G1/G2)
 *   Manual RPM (axis) ...... launcher.setTargetRpm(...)       (RT axis on G1 ONLY)
 *
 * NOTES:
 *   - Gamepad 2 mirrors Gamepad 1 EXCEPT joystick-related mappings:
 *       • NO right-stick-button aim toggle on G2
 *       • NO RT-axis RPM mapping on G2
 *   - See TeleOpAllianceBase “BINDINGS SETUP” for the live button->action table.
 *   - M1/M2 paddles:
 *       If your controller maps paddles to normal buttons (A/B/X/Y/LB/RB), just bind those.
 *       If your hardware exposes unique paddle signals, register them with:
 *         // Example: registerExtraButtonReader(Pad.G1, Btn.M1, () -> { ##read hardware##  });
 *
 * METHODS:
 *   - bindPress(pad, btn, onPress)
 *   - bindHold(pad, btn, whileHeld)
 *   - bindToggle(pad, btn, onEnable, onDisable)
 *   - bindTriggerAxis(pad, trig, consumer0to1)
 *   - bindTriggerPress(pad, trig, onPress)  // thresholded like a digital button
 *   - registerExtraButtonReader(pad, Btn.M1|M2, BooleanSupplier)
 *   - useDefaults(Defaults d)  // optional convenience profile
 *
 * REVISION HISTORY:
 *   2025-10-22  Initial abstraction for Indianola Robotics; limited function list;
 *               LB-first ordering; RB feed; RS aim toggle; G2 mirrors minus joystick.
 * ============================================================================
 */
public class ControllerBindings {

    /* =========================
     * ENUMS / TYPES
     * ========================= */
    public enum Pad { G1, G2 }

    public enum Btn {
        A, B, X, Y,
        LB, RB,
        BACK, START,
        L_STICK_BTN, R_STICK_BTN,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        // Optional “rear paddles” — only active if you register readers
        M1, M2
    }

    public enum Trigger { LT, RT }

    /* =========================
     * INTERNAL STATE
     * ========================= */
    private Gamepad g1, g2;

    private interface Binding { void exec(boolean isDown); }

    private static class PressBinding implements Binding {
        final Runnable onPress; boolean gate=false;
        PressBinding(Runnable r){ this.onPress = r; }
        public void exec(boolean isDown){
            if(isDown && !gate){ gate=true; if(onPress!=null) onPress.run(); }
            if(!isDown){ gate=false; }
        }
    }

    private static class HoldBinding implements Binding {
        final Runnable whileHeld;
        HoldBinding(Runnable r){ this.whileHeld = r; }
        public void exec(boolean isDown){ if(isDown && whileHeld!=null) whileHeld.run(); }
    }

    private static class ToggleBinding implements Binding {
        final Runnable onEnable, onDisable; boolean state=false; boolean gate=false;
        ToggleBinding(Runnable onEnable, Runnable onDisable){ this.onEnable=onEnable; this.onDisable=onDisable; }
        public void exec(boolean isDown){
            if(isDown && !gate){
                gate = true;
                state = !state;
                if(state && onEnable!=null) onEnable.run();
                if(!state && onDisable!=null) onDisable.run();
            }
            if(!isDown){ gate=false; }
        }
    }

    private static class Key {
        final Pad pad; final Btn btn;
        Key(Pad p, Btn b){ pad=p; btn=b; }
        public boolean equals(Object o){ if(!(o instanceof Key)) return false; Key k=(Key)o; return k.pad==pad && k.btn==btn; }
        public int hashCode(){ return Objects.hash(pad, btn); }
    }

    private final Map<Key, Binding> bindings = new HashMap<>();
    private final Map<String, DoubleConsumer> triggerAxes = new HashMap<>();
    private final Map<Key, BooleanSupplier> extraBtnReaders = new HashMap<>();

    private static final double TRIGGER_EDGE_THRESH = 0.5;

    /* =========================
     * PUBLIC BINDING API
     * ========================= */
    public ControllerBindings bindPress(Pad pad, Btn btn, Runnable onPress){
        addBinding(pad, btn, new PressBinding(onPress));
        return this;
    }

    public ControllerBindings bindHold(Pad pad, Btn btn, Runnable whileHeld){
        addBinding(pad, btn, new HoldBinding(whileHeld));
        return this;
    }

    public ControllerBindings bindToggle(Pad pad, Btn btn, Runnable onEnable, Runnable onDisable){
        addBinding(pad, btn, new ToggleBinding(onEnable, onDisable));
        return this;
    }

    public ControllerBindings bindTriggerAxis(Pad pad, Trigger trig, DoubleConsumer fn){
        triggerAxes.put(key(pad, trig), fn);
        return this;
    }

    public ControllerBindings bindTriggerPress(Pad pad, Trigger trig, Runnable onPress){
        // Implement as thresholded edge press using a synthetic binding slot
        Btn pseudo = (trig==Trigger.RT)? Btn.RB : Btn.LB;
        addBinding(pad, pseudo, new PressBinding(onPress){
            @Override public void exec(boolean ignored){
                boolean isDown = readTrigger(pad, trig) > TRIGGER_EDGE_THRESH;
                super.exec(isDown);
            }
        });
        return this;
    }

    /** Register unique readers for rear paddles (M1/M2) IF your hardware exposes them. */
    public ControllerBindings registerExtraButtonReader(Pad pad, Btn btn, BooleanSupplier isDown){
        if(btn!=Btn.M1 && btn!=Btn.M2) throw new IllegalArgumentException("Only M1/M2 supported as extra buttons.");
        extraBtnReaders.put(new Key(pad, btn), isDown);
        return this;
    }

    /** Optional convenience: load a default profile. */
    public interface Defaults {
        void toggleIntake();
        void feedOnce();
        void toggleAim();
        void toggleManualSpeed();
        void setManualRpmFromTrigger(double rt0to1);
    }
    public ControllerBindings useDefaults(Defaults d){
        bindPress(Pad.G1, Btn.LB, d::toggleIntake);
        bindPress(Pad.G1, Btn.RB, d::feedOnce);
        bindToggle(Pad.G1, Btn.R_STICK_BTN, d::toggleAim, d::toggleAim);
        bindToggle(Pad.G1, Btn.Y,  d::toggleManualSpeed, d::toggleManualSpeed);
        bindTriggerAxis(Pad.G1, Trigger.RT, d::setManualRpmFromTrigger);
        return this;
    }

    /** Clear all bindings/readers. */
    public void clear(){
        bindings.clear();
        triggerAxes.clear();
        extraBtnReaders.clear();
    }

    /* =========================
     * UPDATE (CALL EVERY LOOP)
     * ========================= */
    public void update(Gamepad gamepad1, Gamepad gamepad2){
        this.g1 = gamepad1;
        this.g2 = gamepad2;

        // Button bindings
        for(Map.Entry<Key, Binding> e : bindings.entrySet()){
            Key k = e.getKey();
            boolean down = isDown(k.pad, k.btn);
            e.getValue().exec(down);
        }

        // Trigger axes
        triggerAxes.forEach((k, fn) -> {
            Pad pad = k.startsWith("G1")? Pad.G1 : Pad.G2;
            Trigger t = k.endsWith("RT")? Trigger.RT : Trigger.LT;
            fn.accept(readTrigger(pad, t));
        });
    }

    /* =========================
     * HELPERS
     * ========================= */
    private void addBinding(Pad pad, Btn btn, Binding b){ bindings.put(new Key(pad, btn), b); }
    private String key(Pad p, Trigger t){ return p.name()+"_"+t.name(); }

    private boolean isDown(Pad pad, Btn btn){
        // Rear paddles (virtual) via registered readers
        if(btn==Btn.M1 || btn==Btn.M2){
            BooleanSupplier sup = extraBtnReaders.get(new Key(pad, btn));
            return (sup != null) && sup.getAsBoolean();
        }

        Gamepad g = (pad==Pad.G1)? g1 : g2;
        switch(btn){
            case A: return g.a;
            case B: return g.b;
            case X: return g.x;
            case Y: return g.y;
            case LB: return g.left_bumper;
            case RB: return g.right_bumper;
            case BACK: return g.back;
            case START: return g.start;
            case L_STICK_BTN: return g.left_stick_button;
            case R_STICK_BTN: return g.right_stick_button;
            case DPAD_UP: return g.dpad_up;
            case DPAD_DOWN: return g.dpad_down;
            case DPAD_LEFT: return g.dpad_left;
            case DPAD_RIGHT: return g.dpad_right;
            default: return false;
        }
    }

    private double readTrigger(Pad pad, Trigger t){
        Gamepad g = (pad==Pad.G1)? g1 : g2;
        return (t==Trigger.RT)? g.right_trigger : g.left_trigger;
    }
}
