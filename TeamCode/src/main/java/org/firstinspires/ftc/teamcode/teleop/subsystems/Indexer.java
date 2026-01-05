package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.*;

@Config
public class Indexer {

    /* ================= CONFIG ================= */

    public String motifPattern = "PPG";

    public static double kickerLeftDown  = 0.705;
    public static double kickerLeftUp    = 0.45;
    public static double kickerRightDown = 0.63;
    public static double kickerRightUp   = 0.37;
    public static double kickerBackDown  = 0.60;
    public static double kickerBackUp    = 0.34;

    public static double kickerSleep = 0.25;
    public static double shootSleep  = 0.03;

    static final double DISTANCE_THRESHOLD_MM = 28.0;
    public int gain = 20;

    /* ================= HOLDERS ================= */

    public final Holder rightHolder;
    public final Holder leftHolder;
    public final Holder backHolder;

    public final Holder[] holders;

    /* ================= INIT ================= */

    public Indexer(OpMode opMode) {
        rightHolder = new Holder(
                opMode,
                "rightKicker",
                "colorRR", "colorRL",
                kickerRightUp, kickerRightDown,
                gain
        );

        leftHolder = new Holder(
                opMode,
                "leftKicker",
                "colorLL", "colorLR",
                kickerLeftUp, kickerLeftDown,
                gain
        );

        backHolder = new Holder(
                opMode,
                "backKicker",
                "colorBR", "colorBL",
                kickerBackUp, kickerBackDown,
                gain
        );

        holders = new Holder[]{ rightHolder, leftHolder, backHolder };

        resetIndexer();
    }

    /* ================= INDEXER-LEVEL ================= */

    public void resetIndexer() {
        for (Holder h : holders) h.down();
    }

    public int countBalls() {
        int balls = 0;
        for (Holder h : holders) {
            if (h.ballPresent()) balls++;
        }
        return balls;
    }

    /* ================= BASIC ACTIONS ================= */

    public Action shootRight() { return rightHolder.kickResetAction(); }
    public Action shootLeft()  { return leftHolder.kickResetAction(); }
    public Action shootBack()  { return backHolder.kickResetAction(); }

    public Action shootAllSequential() {
        List<Action> actions = new ArrayList<>();
        for (Holder h : holders) {
            actions.add(h.kickResetAction());
            actions.add(new SleepAction(shootSleep));
        }
        return new SequentialAction(actions.toArray(new Action[0]));
    }

    /* ================= SHOOT BY COLOR ================= */

    /**
     * Shoots ONE green if any holder currently contains GREEN, otherwise no-op.
     */
    public Action shootGreen() {
        Holder h = findFirstHolderWithColor("GREEN");
        if (h == null) return new InstantAction(() -> {});
        return h.kickResetAction();
    }

    /**
     * Shoots ONE purple if any holder currently contains PURPLE, otherwise no-op.
     */
    public Action shootPurple() {
        Holder h = findFirstHolderWithColor("PURPLE");
        if (h == null) return new InstantAction(() -> {});
        return h.kickResetAction();
    }

    private Holder findFirstHolderWithColor(String color) {
        for (Holder h : holders) {
            if (color.equals(h.getColor())) return h;
        }
        return null;
    }

    /* ================= MOTIF SHOOT (NO SKIPS, MAXIMIZE MOTIFS) ================= */

    /**
     * IMPORTANT: We do NOT "skip" motif characters.
     * We pick an order to shoot AVAILABLE balls such that, as a stream,
     * we maximize completed motif occurrences (using overlap-aware matching).
     *
     * If we don't have the right colors to complete the motif, we'll still choose
     * the best order to maximize completions (and then partial progress as tie-breaker).
     */
    public void shootMotif() {
        List<Integer> order = planMotifOrder(motifPattern);

        for (int idx : order) {
            Holder h = holders[idx];

            h.up();
            sleepMillis((long) (kickerSleep * 1000));
            h.down();

            sleepMillis((long) (shootSleep * 1000));
        }
    }

    /**
     * RoadRunner Action version of motif shooting using the same maximizing logic.
     */
    public Action shootMotifAction() {
        List<Integer> order = planMotifOrder(motifPattern);
        if (order.isEmpty()) return new InstantAction(() -> {});

        List<Action> actions = new ArrayList<>();
        for (int idx : order) {
            Holder h = holders[idx];
            actions.add(new InstantAction(h::up));
            actions.add(new SleepAction(kickerSleep));
            actions.add(new InstantAction(h::down));
            actions.add(new SleepAction(shootSleep));
        }
        return new SequentialAction(actions.toArray(new Action[0]));
    }

    /**
     * Choose a firing order (permutation of available holders) that maximizes:
     *  1) number of completed motifs in the shot stream (overlap-aware)
     *  2) remaining motif progress at end (higher is better)
     *  3) number of shots (higher is better) (usually fixed, but kept for stability)
     */
    private List<Integer> planMotifOrder(String motif) {
        // Build list of available holder indices and their current colors
        ArrayList<Integer> available = new ArrayList<>();
        String[] colorByIdx = new String[holders.length];

        for (int i = 0; i < holders.length; i++) {
            String c = holders[i].getColor();   // "GREEN", "PURPLE", "EMPTY", "UNKNOWN"
            colorByIdx[i] = c;
            if (!"EMPTY".equals(c)) available.add(i);
        }

        if (available.isEmpty()) return Collections.emptyList();

        // Build overlap-aware automaton for motif (KMP)
        MotifAutomaton auto = new MotifAutomaton(motif);

        // Brute force all permutations of up to 3 holders (safe + simple)
        BestOrder best = new BestOrder();
        permute(available, 0, auto, colorByIdx, best);

        return best.bestOrder != null ? best.bestOrder : Collections.emptyList();
    }

    private static class BestOrder {
        List<Integer> bestOrder = null;
        int bestMotifs = -1;
        int bestProgress = -1;
        int bestShots = -1;
    }

    private void permute(ArrayList<Integer> list, int start, MotifAutomaton auto, String[] colorByIdx, BestOrder best) {
        if (start == list.size()) {
            Score s = scoreOrder(list, auto, colorByIdx);
            if (isBetter(s, best)) {
                best.bestMotifs = s.motifs;
                best.bestProgress = s.progress;
                best.bestShots = s.shots;
                best.bestOrder = new ArrayList<>(list);
            }
            return;
        }

        for (int i = start; i < list.size(); i++) {
            Collections.swap(list, start, i);
            permute(list, start + 1, auto, colorByIdx, best);
            Collections.swap(list, start, i);
        }
    }

    private static class Score {
        final int motifs;
        final int progress;
        final int shots;

        Score(int motifs, int progress, int shots) {
            this.motifs = motifs;
            this.progress = progress;
            this.shots = shots;
        }
    }

    private Score scoreOrder(List<Integer> order, MotifAutomaton auto, String[] colorByIdx) {
        int state = 0;     // how many chars matched so far
        int motifs = 0;

        for (int idx : order) {
            char shotChar = toMotifChar(colorByIdx[idx]); // 'P', 'G', or 'X'
            MotifAutomaton.StepResult r = auto.step(state, shotChar);
            state = r.nextState;
            motifs += r.completed;
        }

        return new Score(motifs, state, order.size());
    }

    private boolean isBetter(Score s, BestOrder best) {
        if (s.motifs != best.bestMotifs) return s.motifs > best.bestMotifs;
        if (s.progress != best.bestProgress) return s.progress > best.bestProgress;
        return s.shots > best.bestShots;
    }

    private char toMotifChar(String color) {
        if ("PURPLE".equals(color)) return 'P';
        if ("GREEN".equals(color)) return 'G';
        return 'X'; // UNKNOWN or anything else acts like mismatch
    }

    /**
     * Overlap-aware motif matcher using KMP prefix function.
     * Lets streams like "P P G P" still count motifs correctly with overlaps.
     */
    private static class MotifAutomaton {
        final char[] pat;
        final int[] pi;

        MotifAutomaton(String motif) {
            this.pat = motif.toCharArray();
            this.pi = buildPrefix(pat);
        }

        static int[] buildPrefix(char[] p) {
            int[] pi = new int[p.length];
            int j = 0;
            for (int i = 1; i < p.length; i++) {
                while (j > 0 && p[i] != p[j]) j = pi[j - 1];
                if (p[i] == p[j]) j++;
                pi[i] = j;
            }
            return pi;
        }

        static class StepResult {
            final int nextState;
            final int completed;
            StepResult(int nextState, int completed) {
                this.nextState = nextState;
                this.completed = completed;
            }
        }

        StepResult step(int state, char c) {
            int j = state;

            while (j > 0 && (j >= pat.length || pat[j] != c)) {
                j = pi[j - 1];
            }

            if (j < pat.length && pat[j] == c) j++;

            int completed = 0;
            if (j == pat.length) {
                completed = 1;
                j = pi[pat.length - 1]; // allow overlap
            }

            return new StepResult(j, completed);
        }
    }

    private void sleepMillis(long ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }

    /* =====================================================
       ======================= HOLDER =======================
       ===================================================== */

    public static class Holder {

        private final Servo kicker;
        private final RevColorSensorV3 sensorA;
        private final RevColorSensorV3 sensorB;

        private final double upPos;
        private final double downPos;

        private final float[] hsv = new float[3];

        public Holder(
                OpMode opMode,
                String kickerServoName,
                String leftSensorName,
                String rightSensorName,
                double upPos,
                double downPos,
                int gain
        ) {
            kicker = opMode.hardwareMap.servo.get(kickerServoName);

            sensorA = opMode.hardwareMap.get(RevColorSensorV3.class, leftSensorName);
            sensorB = opMode.hardwareMap.get(RevColorSensorV3.class, rightSensorName);

            this.upPos = upPos;
            this.downPos = downPos;

            sensorA.setGain(gain);
            sensorB.setGain(gain);
        }

        /* ================= SERVO ================= */

        public void up()   { kicker.setPosition(upPos); }
        public void down() { kicker.setPosition(downPos); }

        private void kick()  { up(); }
        private void reset() { down(); }

        /* ================= ACTION ================= */

        public Action kickResetAction() {
            return new SequentialAction(
                    new InstantAction(this::kick),
                    new SleepAction(Indexer.kickerSleep),
                    new InstantAction(this::reset)
            );
        }

        /* ================= DISTANCE ================= */

        private double safeDistance(RevColorSensorV3 s) {
            double d = s.getDistance(DistanceUnit.MM);
            return (Double.isNaN(d) || Double.isInfinite(d)) ? -1 : d;
        }

        public boolean ballPresent() {
            double da = safeDistance(sensorA);
            double db = safeDistance(sensorB);
            return (da > 0 && da < DISTANCE_THRESHOLD_MM)
                    || (db > 0 && db < DISTANCE_THRESHOLD_MM);
        }

        /* ================= COLOR ================= */

        private float getHue(RevColorSensorV3 sensor) {
            android.graphics.Color.RGBToHSV(
                    sensor.red(), sensor.green(), sensor.blue(), hsv
            );
            return hsv[0];
        }

        private boolean isGreenHue(float h)  { return h > 160 && h < 180; }
        private boolean isPurpleHue(float h) { return h > 180 && h < 225; }

        public String getColor() {
            if (!ballPresent()) return "EMPTY";

            float h1 = getHue(sensorA);
            float h2 = getHue(sensorB);

            if (isGreenHue(h1) || isGreenHue(h2))   return "GREEN";
            if (isPurpleHue(h1) || isPurpleHue(h2)) return "PURPLE";

            return "UNKNOWN";
        }
    }
}
